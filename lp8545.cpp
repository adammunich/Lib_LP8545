/**
* @file lp8545.cpp
*
* @brief LP8545 Driver
*
* @copyright
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation; either
* version 3.0 of the License, or (at your option) any later version.
*
* @copyright
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* @author Arduino LLC
* @author Adam Munich
*/

// ##################################################################################
// Includes
// ----------------------------------------------------------------------------------
#include "./LP8545.h"
#include "../pin_settings.h"

#include <wiring_private.h>
#include <Wire.h>


// ##################################################################################
// Macros used in this CPP file
// ----------------------------------------------------------------------------------
#ifndef VARIANT_MCK
#error Must define CPU frequency in VARIANT_MCK
#endif

#ifndef lowByte(w)
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#endif

#ifndef highByte(w)
#define highByte(w) ((uint8_t) ((w) >> 8))
#endif

#ifndef countBytes(a)
#define countBytes(a) (sizeof(a)/sizeof(a[0]))
#endif

#define EASING_STEPS 41
#define EASING_IRQ_FREQUENCY 50

// ##################################################################################
// Global variables
// ----------------------------------------------------------------------------------

#ifdef _VARIANT_ARDUINO_ZERO_

// PWM generation timers
EPioType g_LP_pinPwm_EPioType;
bool g_LP_pinPwm_oneShotModeEnabled;
uint8_t g_LP_pinPwm_tcChannel;
LP_pinPwm_timerType_t g_LP_pinPwm_timerType;
Tc * g_LP_pinPwm_TCx;
Tcc * g_LP_pinPwm_TCCx;

// Choose vsync timer (TCC2 is not used by important pins)
Tcc * g_LP_vsyncTCC = TCC2;
IRQn g_LP_vsyncTCC_handler = TCC2_IRQn;
bool g_LP_vsyncTCC_isRunning = false;

// Choose easing timer (TC4 is least important...)
Tc * g_LP_easingTC = TC4;
IRQn g_LP_easingTC_handler = TC4_IRQn;
bool g_LP_easingTC_isRunning = false;
float easingFunctionLut[EASING_STEPS];
#endif

// Pins
typedef struct
{
	uint32_t pwmPin;
	uint32_t vsyncIrqPin;
} pinId_t;

pinId_t g_LP_pinId;

// Brightness state
typedef struct 
{
	int16_t startingValue;
	int16_t targetValue;
	int16_t currentValue;
	int16_t lastBrightnessSetting;
	
	int16_t easingValue;	
	int8_t easingDirection;
	int8_t easingLutIndex;
	
	bool haltSemaphore;
	bool boostEnabled;
	bool advancedSlopeEnabled;
} brightnessState_t;

brightnessState_t g_LP_brightnessState;


// ##################################################################################
// Function definitions
// ----------------------------------------------------------------------------------
#ifdef _VARIANT_ARDUINO_ZERO_

static void setTCXx_CCx(uint16_t brightnessValue)
{
	//setPwmPinTimer(g_LP_pinId.pwmPin, cie_12bit[g_LP_brightnessState.targetValue]);
	switch(g_LP_pinPwm_timerType)
	{
		case pinPwm_timerType::timerTypeTCC:
		{
			g_LP_pinPwm_TCCx->CC[g_LP_pinPwm_tcChannel].reg = g_LP_cie_12bit[brightnessValue];
			while (g_LP_pinPwm_TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
			break;
		}
		case pinPwm_timerType::timerTypeTC:
		{
			g_LP_pinPwm_TCx->COUNT8.CC[g_LP_pinPwm_tcChannel].reg = g_LP_cie_8bit[brightnessValue];
			while (g_LP_pinPwm_TCx->COUNT8.STATUS.bit.SYNCBUSY);
			break;
		}
	}
}

void TCC2_Handler(void)
{
	// If this interrupt is due to the compare register matching the timer count
	// we toggle the LED.
	
	// Capture compare 0
	if (g_LP_vsyncTCC->INTFLAG.bit.MC0 == 1)
	{
		g_LP_vsyncTCC->INTFLAG.bit.MC0 = 1;
		pinPeripheral(g_LP_pinId.pwmPin, g_LP_pinPwm_EPioType);
	}

	// Capture compare 1
	if (g_LP_vsyncTCC->INTFLAG.bit.MC1 == 1)
	{
		g_LP_vsyncTCC->INTFLAG.bit.MC1 = 1;
		
		if(g_LP_pinPwm_oneShotModeEnabled)
		{
			g_LP_vsyncTCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
		}
		pinMode(g_LP_pinId.pwmPin, OUTPUT);
		digitalWrite(g_LP_pinId.pwmPin, LOW);
	}
}

void TC4_Handler(void)
{
	// If this interrupt is due to the compare register matching the timer count
	// we toggle the LED.
	
	// Capture compare 0
	if (g_LP_easingTC->COUNT16.INTFLAG.bit.MC0 == 1)
	{
		g_LP_easingTC->COUNT16.INTFLAG.bit.MC0 = 1;

		g_LP_easingTC->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;
		g_LP_easingTC->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;

		// This block eases the brightness change
		if((g_LP_brightnessState.currentValue != g_LP_brightnessState.targetValue) && !(g_LP_brightnessState.haltSemaphore))
		{
			// Fading down
			if(g_LP_brightnessState.easingDirection < 0)
			{
				uint16_t easingDifference = g_LP_brightnessState.startingValue - g_LP_brightnessState.targetValue;
				g_LP_brightnessState.easingValue = easingDifference * easingFunctionLut[g_LP_brightnessState.easingLutIndex];
				g_LP_brightnessState.easingLutIndex -= 1;
								
				if(g_LP_brightnessState.easingLutIndex == 0)
				{
					g_LP_brightnessState.currentValue = g_LP_brightnessState.targetValue;				
				}
				else
				{
					g_LP_brightnessState.currentValue = g_LP_brightnessState.targetValue + g_LP_brightnessState.easingValue;
				}
			}
			else  // Fading up
			{
				uint16_t easingDifference = g_LP_brightnessState.targetValue - g_LP_brightnessState.startingValue;
				g_LP_brightnessState.easingValue = easingDifference * easingFunctionLut[g_LP_brightnessState.easingLutIndex];
				g_LP_brightnessState.easingLutIndex += 1;
				
				if(g_LP_brightnessState.easingLutIndex == EASING_STEPS - 1)
				{
					g_LP_brightnessState.currentValue = g_LP_brightnessState.targetValue;
				}
				else
				{
					g_LP_brightnessState.currentValue = g_LP_brightnessState.startingValue + g_LP_brightnessState.easingValue;
				}				
			}
			setTCXx_CCx(g_LP_brightnessState.currentValue);
		}
	}
}

void VSYNC_IRQ(void)
{
	if(g_LP_vsyncTCC_isRunning)
	{
		g_LP_vsyncTCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
		g_LP_vsyncTCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
	}
	else
	{
		g_LP_vsyncTCC->CTRLA.bit.ENABLE = 1;
		g_LP_vsyncTCC_isRunning = true;
	}
}
#endif


// ##################################################################################
// LP8545_ Class
// ----------------------------------------------------------------------------------
// Sensor object
LP8545_ g_LP8545;

LP8545_::LP8545_(void)
{
}

uint8_t LP8545_ :: begin(uint32_t pwmPin, uint32_t vsyncInterruptPin)
{
	// Disable output
	powerOff();
	
	// Set defaults
	g_LP_brightnessState.currentValue = 0;
	g_LP_brightnessState.startingValue = 0;
	g_LP_brightnessState.targetValue = 0;
	g_LP_brightnessState.lastBrightnessSetting = 0;
	g_LP_brightnessState.easingDirection = +1;
	g_LP_brightnessState.easingLutIndex = 0;
	g_LP_brightnessState.haltSemaphore = false;	
	g_LP_brightnessState.boostEnabled = false;
	
	// Set pins
	g_LP_pinId.pwmPin = pwmPin;
	g_LP_pinId.vsyncIrqPin = vsyncInterruptPin;
	
	// Pin modes
	pinMode(g_LP_pinId.pwmPin, OUTPUT);
	pinMode(g_LP_pinId.vsyncIrqPin, INPUT_PULLUP);

	// Pin default states
	digitalWrite(g_LP_pinId.pwmPin, LOW);

	// Interrupt for one-shot		
	#ifdef _VARIANT_ARDUINO_ZERO_	
	attachInterrupt(g_LP_pinId.vsyncIrqPin, VSYNC_IRQ, RISING);
	#endif
}

void LP8545_ :: setup(uint16_t ledMaxCurrentuA, uint8_t ledVoltage, LP_easingFunctionType_t easingType, LP_config_vsyncType_t vsyncType, uint8_t vsyncFrequency, uint16_t vsyncStartDelay, uint16_t vsyncPulseLength)
{
	bool startNow;
	
	// Vsync Type
	switch(vsyncType)
	{
		case vsyncModeSetup::vsyncModeOneshot:
		{
			g_LP_pinPwm_oneShotModeEnabled = true;
			g_LP_vsyncTCC_isRunning = true;
			startNow = true;
			break;			
		}
		case vsyncModeSetup::vsyncModeAstableTriggered:
		{
			g_LP_pinPwm_oneShotModeEnabled = false;
			g_LP_vsyncTCC_isRunning = false;
			startNow = false;
			break;			
		}
		case vsyncModeSetup::vsyncModeAstableAutostart:
		{
			g_LP_pinPwm_oneShotModeEnabled = false;
			g_LP_vsyncTCC_isRunning = true;
			startNow = true;			
			break;
		}
	}
	
	// LP_REGISTER_DEVICE_CTL
	#ifdef _VARIANT_ARDUINO_ZERO_
	setBrtMode(LP_brtModeConfig_t::brtModeDirectControl);
	#else // NOT SAMD21
	setBrtMode(LP_brtModeConfig_t::brtModePwmInputDutyCycleControl);	
	#endif
	
	// LP_REGISTER_EEPROM_0
	setCurrentMicroamps(ledMaxCurrentuA);
	
	// LP_REGISTER_EEPROM_1
	setBoostFreq(boostFreqConfig::boostFreq625kHz);
	setLedFaultDetection(ledFaultsConfig::ledFaultsDisabled);
	setTempLimit(tempLimitConfig::tempLimit120C);
	setSlope(slopeConfig::slopeDisabled, advancedSlopeConfig::advancedSlopeDisabled);
	
	// LP_REGISTER_EEPROM_2
	setAdaptiveSpeed(adaptiveSpeedConfig1::adaptiveSpeedNormalMode, adaptiveSpeedConfig2::adaptiveSpeedOncePerCycle);
	setFetEnable(enExternalFetConfig::externalFetEnabled);
	setAdaptiveMode(enAdaptiveModeConfig::adaptiveModeEnabled);
	setBoostOnOff(enableBoostConfig::boostEnabled);
	setBoostCurrent(boostMaxCurrentConfig::maxBoostCurrent2500mA);
	
	// LP_REGISTER_EEPROM_3
	setUVLO(LP_uvloConfig_t::uvloDisabled);
	setPhaseShiftPWM(LP_enPhaseShiftPwmConfig_t::phaseShiftPWMEnable);
	setPWMFreq(20000);
	
	// LP_REGISTER_EEPROM_4
	setPwmResolution(LP_pwmResolutionConfig_t::pwmResolutionMEDIUM);
	setIresEnable(LP_enIresConfig_t::iresDisabled);
	setLedFaultThreshold (LP_ledFaultThresholdConfig_t::ledFaultThreshold2v3);
	setLedFaultHeadrooom (LP_ledFaultHeadroomConfig_t::ledHeadroomFault125mV);
	
	// LP_REGISTER_EEPROM_5
	setVsync(LP_enVsyncConfig_t::vsyncDisabled);
	setDither(LP_ditherConfig_t::ditherDisabled);
	setVboost(ledVoltage);
	
	// LP_REGISTER_EEPROM_6
	setPll(2267);
	setFresEnabled(LP_enFresConfig_t::fresDisabled);
	setHysteresisResolution(LP_hysteresisConfig_t::hysteresisDisabled);
	
	// Start vsync timer
	#ifdef _VARIANT_ARDUINO_ZERO_
		startVsyncTimer(g_LP_pinPwm_oneShotModeEnabled, vsyncFrequency, vsyncStartDelay, vsyncPulseLength, startNow);
		pinMode(g_LP_pinId.pwmPin, OUTPUT);
		setPwmPinTimer(g_LP_pinId.pwmPin, g_LP_cie_12bit[255]);
	#else // NOT SAMD21	
		pinMode(g_LP_pinId.pwmPin, OUTPUT);
		analogWrite(g_LP_pinId.pwmPin, 0);
		g_LP_brightnessState.advancedSlopeEnabled = false;
	#endif
	
	// Start easing timer
	#ifdef _VARIANT_ARDUINO_ZERO_
	// (Re?)Make easing lookup table
	for(uint8_t i = 0; i < EASING_STEPS; i++)
	{
		float easingFunctionIndex = i * ((float) 1 / (EASING_STEPS - 1));
		easingFunctionLut[i] = easingFunction(easingFunctionIndex, easingType);
	}
	
	startEasingTimer(EASING_IRQ_FREQUENCY);
	#else
		g_LP_brightnessState.advancedSlopeEnabled = false;
	#endif
	
	// Start P.W.M.
	#ifdef _VARIANT_ARDUINO_ZERO_
		pinMode(g_LP_pinId.pwmPin, OUTPUT);
		setPwmPinTimer(g_LP_pinId.pwmPin, g_LP_cie_12bit[255]);
	#else // NOT SAMD21
		pinMode(g_LP_pinId.pwmPin, OUTPUT);
		analogWrite(g_LP_pinId.pwmPin, 0);
	#endif
}

void LP8545_ :: loopTask(void)
{
}

void LP8545_ :: end(void)
{
	// Disable output
	setBlCtl(LP_blCtlConfig_t::blCtlOff);
	setBoostOnOff(enableBoostConfig::boostDisabled);
	
	// Detatch NVIC
	detachInterrupt(PIN_backlight_vsync_in);
}

void LP8545_ :: brightness(uint16_t pwmVal, bool shouldFade)
{
	pwmVal = constrain(pwmVal, 0, 512);
	
	#ifdef _VARIANT_ARDUINO_ZERO_
	if(shouldFade == true)
	{
		g_LP_brightnessState.haltSemaphore = true;
		
		if(pwmVal < g_LP_brightnessState.currentValue)
		{
			g_LP_brightnessState.easingDirection = -1;
			g_LP_brightnessState.easingLutIndex = EASING_STEPS - 1;
		}
		else if(pwmVal > g_LP_brightnessState.currentValue)
		{
			g_LP_brightnessState.easingDirection = +1;
			g_LP_brightnessState.easingLutIndex = 0;
		}
		else // No change
		{
			g_LP_brightnessState.easingDirection = 0;
		}

		g_LP_brightnessState.startingValue = g_LP_brightnessState.currentValue;
		g_LP_brightnessState.targetValue = pwmVal;
		g_LP_brightnessState.lastBrightnessSetting = pwmVal;
		
		g_LP_brightnessState.haltSemaphore = false;
	}
	else
	{
		g_LP_brightnessState.haltSemaphore = true;
		
		g_LP_brightnessState.targetValue = pwmVal;
		g_LP_brightnessState.startingValue = g_LP_brightnessState.currentValue;
		g_LP_brightnessState.currentValue = pwmVal;
		g_LP_brightnessState.lastBrightnessSetting = pwmVal;
		
		setTCXx_CCx(g_LP_brightnessState.currentValue);
						
		g_LP_brightnessState.haltSemaphore = false;
	}
	
	#else // NOT SAMD21	

	if(shouldFade == true && g_LP_brightnessState.advancedSlopeEnabled == false)
	{
		setSlope(LP_slopeConfig_t::slope500ms, LP_advancedSlopeConfig_t::advancedSlopeEnabled);
		g_LP_brightnessState.advancedSlopeEnabled = true;
	}
	else if(shouldFade == false)
	{
		setSlope(LP_slopeConfig_t::slopeDisabled, LP_advancedSlopeConfig_t::defaultValue);
		g_LP_brightnessState.advancedSlopeEnabled = false;
	}
	
	g_LP_brightnessState.currentValue = pwmVal;
	g_LP_brightnessState.startingValue = pwmVal;
	g_LP_brightnessState.lastBrightnessSetting = pwmVal;	
	analogWrite(g_LP_pinId.pwmPin, g_LP_cie_8bit[g_LP_brightnessState.currentValue]);

	#endif
}

void LP8545_ :: powerOn(void)
{
	// Enable boost
	setBlCtl(blCtlConfig::blCtlOn);
	setBoostOnOff(enableBoostConfig::boostEnabled);		
	g_LP_brightnessState.boostEnabled = true;
}

void LP8545_ :: powerOff(void)
{
	// Disable boost
	setBlCtl(blCtlConfig::blCtlOff);
	setBoostOnOff(enableBoostConfig::boostDisabled);				
	g_LP_brightnessState.boostEnabled = false;
}

void LP8545_ :: lightOn(bool shouldFade)
{		
	#ifdef _VARIANT_ARDUINO_ZERO_
	
	// Set target and stuff.
	if(shouldFade == true)
	{
		g_LP_brightnessState.haltSemaphore = true;

		// Get last brightness
		g_LP_brightnessState.targetValue = g_LP_brightnessState.lastBrightnessSetting;
		g_LP_brightnessState.startingValue = g_LP_brightnessState.currentValue;

		// Set direction
		g_LP_brightnessState.easingDirection = +1;
		g_LP_brightnessState.easingLutIndex = 0;
		
		g_LP_brightnessState.haltSemaphore = false;
	}
	else
	{
		g_LP_brightnessState.haltSemaphore = true;
		
		// Set target values
		g_LP_brightnessState.targetValue = g_LP_brightnessState.lastBrightnessSetting;
		g_LP_brightnessState.currentValue = g_LP_brightnessState.lastBrightnessSetting;
		
		setTCXx_CCx(g_LP_brightnessState.currentValue);
				
		g_LP_brightnessState.haltSemaphore = false;
	}

	#else // NOT SAMD21	
	
	if(shouldFade == true && g_LP_brightnessState.advancedSlopeEnabled == false)
	{
		setSlope(LP_slopeConfig_t::slope500ms, LP_advancedSlopeConfig_t::advancedSlopeEnabled);
		g_LP_brightnessState.advancedSlopeEnabled = true;
	}
	else if(shouldFade == false)
	{
		setSlope(LP_slopeConfig_t::slopeDisabled, LP_advancedSlopeConfig_t::defaultValue);
		g_LP_brightnessState.advancedSlopeEnabled = false;
	}
		
	g_LP_brightnessState.currentValue = g_LP_brightnessState.lastBrightnessSetting;
	
	analogWrite(g_LP_pinId.pwmPin, g_LP_cie_8bit[g_LP_brightnessState.currentValue]);
	#endif
}

void LP8545_ :: lightOff(bool shouldFade)
{		
	#ifdef _VARIANT_ARDUINO_ZERO_
		
	// Set target and stuff.
	if(shouldFade == true)
	{
		g_LP_brightnessState.haltSemaphore = true;

		// Set target values
		g_LP_brightnessState.startingValue = g_LP_brightnessState.currentValue;
		g_LP_brightnessState.targetValue = 0;

		// Set direction
		g_LP_brightnessState.easingDirection = -1;
		g_LP_brightnessState.easingLutIndex = EASING_STEPS - 1;

		g_LP_brightnessState.haltSemaphore = false;
	}
	else
	{
		g_LP_brightnessState.haltSemaphore = true;

		g_LP_brightnessState.startingValue	= g_LP_brightnessState.currentValue;

		g_LP_brightnessState.targetValue = 0;
		g_LP_brightnessState.currentValue = 0;

		setTCXx_CCx(g_LP_brightnessState.currentValue);

		g_LP_brightnessState.haltSemaphore = false;
	}
	
	#else // NOT SAMD21	
		
	if(shouldFade == true && g_LP_brightnessState.advancedSlopeEnabled == false)
	{
		setSlope(LP_slopeConfig_t::slope500ms, LP_advancedSlopeConfig_t::advancedSlopeEnabled);
		g_LP_brightnessState.advancedSlopeEnabled = true;
	}
	else if(shouldFade == false)
	{
		setSlope(LP_slopeConfig_t::slopeDisabled, LP_advancedSlopeConfig_t::defaultValue);
		g_LP_brightnessState.advancedSlopeEnabled = false;
	}
		
	g_LP_brightnessState.startingValue = g_LP_brightnessState.currentValue;
	g_LP_brightnessState.currentValue = 0;
	analogWrite(g_LP_pinId.pwmPin, g_LP_brightnessState.currentValue);
		
	#endif
}


// ----------------------------------------------------------------------------------
// Private
// ----------------------------------------------------------------------------------
// Hardware abstraction layer
void LP8545_ :: setPwmPinTimer(uint32_t pin, uint32_t value)
{
	#ifdef _VARIANT_ARDUINO_ZERO_
	uint32_t attr = g_APinDescription[pin].ulPinAttribute;

	if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
	{
		uint32_t tcNum = GetTCNumber(g_APinDescription[pin].ulPWMChannel);
		g_LP_pinPwm_tcChannel = GetTCChannelNumber(g_APinDescription[pin].ulPWMChannel);
		static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

		if (attr & PIN_ATTR_TIMER)
		{
			#if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
			// Compatibility for cores based on SAMD core <=1.6.2
			if (g_APinDescription[pin].ulPinType == PIO_TIMER_ALT)
			{
				pinPeripheral(pin, PIO_TIMER_ALT);
				g_LP_pinPwm_EPioType = PIO_TIMER_ALT;
			}
			else
			#endif
			{
				pinPeripheral(pin, PIO_TIMER);
				g_LP_pinPwm_EPioType = PIO_TIMER;
			}
		}
		else
		{
			// We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
			pinPeripheral(pin, PIO_TIMER_ALT);
			g_LP_pinPwm_EPioType = PIO_TIMER_ALT;
		}

		if (!tcEnabled[tcNum])
		{
			tcEnabled[tcNum] = true;

			uint16_t GCLK_CLKCTRL_IDs[] =
			{
				GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
				GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
				GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
				GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
				GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
				GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
				GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
				GCLK_CLKCTRL_ID(GCM_TC6_TC7)    // TC7
			};
			
			GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
			while (GCLK->STATUS.bit.SYNCBUSY == 1);

			// Set PORT
			if (tcNum >= TCC_INST_NUM)
			{
				// This timer only supports 8 bit periods, sad panda, reduce PWM resolution...
				value = mapResolution(value, 12, 8);
				
				// -- Configure TC
				Tc * TCx = (Tc*) GetTC(g_APinDescription[pin].ulPWMChannel);
				
				// Set global for later reference
				g_LP_pinPwm_timerType = pinPwm_timerType::timerTypeTC;
				g_LP_pinPwm_TCx = TCx;
				
				// Disable timer-counter[x]
				TCx->COUNT8.CTRLA.bit.ENABLE = 0;
				while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
				
				// Set timer-counter[x] mode to 8 bits, normal PWM, pre-scaler 1/4
				TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8;
				while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
				
				// Set timer-counter[x] as normal PWM
				TCx->COUNT8.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
				while (TCx->COUNT8.STATUS.bit.SYNCBUSY);

				// Set timer-counter[x] pre-scaler to 1/4
				TCx->COUNT8.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4;
				while (TCx->COUNT8.STATUS.bit.SYNCBUSY);

				// Set the initial value
				TCx->COUNT8.CC[g_LP_pinPwm_tcChannel].reg = (uint8_t) value;
				while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
				
				// Set PER to maximum counter value (resolution : 0xFF, 255)
				TCx->COUNT8.PER.reg = 0xFF;
				while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
				
				// Enable timer-counter[x]
				TCx->COUNT8.CTRLA.bit.ENABLE = 1;
				while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
			}
			else
			{
				// This timer supports > 8 bit periods!
				value = mapResolution(value, 12, 12);

				// -- Configure timer-control-counter[x]
				Tcc * TCCx = (Tcc*) GetTC(g_APinDescription[pin].ulPWMChannel);

				// Set timer-control-counter global for later reference
				g_LP_pinPwm_timerType = pinPwm_timerType::timerTypeTCC;
				g_LP_pinPwm_TCCx = TCCx;

				// Disable timer-control-counter[x]
				TCCx->CTRLA.bit.ENABLE = 0;
				while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

				// Set pre-scaler to 1/1
				TCCx->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;
				while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

				// Set timer-control-counter as normal PWM
				TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
				while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

				// Set the initial value
				TCCx->CC[g_LP_pinPwm_tcChannel].reg = (uint32_t) value;
				while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
				
				// Set PER to maximum counter value (resolution : 12 Bits, 4096)
				TCCx->PER.reg = 0xFFF;
				while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
				
				// Enable timer-control-counter[x]
				TCCx->CTRLA.bit.ENABLE = 1;
				while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
			}
		}
	}
	
	#else // NOT SAMD21	
	#error "LP8545 timer functions only support SAMD21G/J, uncomment to proceed..."
	
	analogWrite(pin, g_LP_cie_8bit[value]);
	
	#endif
}

void LP8545_ :: startVsyncTimer(bool oneShotEnabled, uint8_t timerFrequency, uint16_t pulseStart, uint16_t pulseLength, bool startNow)
{
	#ifdef _VARIANT_ARDUINO_ZERO_
	uint16_t GCLK_CLKCTRL_IDx;
	
	if(g_LP_vsyncTCC == TCC0)
	{
		GCLK_CLKCTRL_IDx = 	GCLK_CLKCTRL_ID(GCM_TCC0_TCC1);
	}
	else if(g_LP_vsyncTCC == TCC1)
	{
		GCLK_CLKCTRL_IDx = 	GCLK_CLKCTRL_ID(GCM_TCC0_TCC1);
	}
	else if(g_LP_vsyncTCC == TCC2)
	{
		GCLK_CLKCTRL_IDx = 	GCLK_CLKCTRL_ID(GCM_TCC2_TC3);
	}

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDx);
	while (GCLK->STATUS.bit.SYNCBUSY == 1);

	g_LP_vsyncTCC->INTENCLR.bit.MC0 = 1;
	g_LP_vsyncTCC->INTENCLR.bit.MC1 = 1;

	g_LP_vsyncTCC->CTRLA.bit.ENABLE = 0;
	
	if(oneShotEnabled)
	{
		g_LP_vsyncTCC->CTRLBSET.bit.ONESHOT = 1;
		while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
	}

	// Set pre-scaler to 16
	uint16_t TIMER_PRESCALER_DIV = 16;
	g_LP_vsyncTCC->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV16;
	while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

	// Set config mode
	// @todo test: is this needed?
	g_LP_vsyncTCC->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;
	while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

	// Go from microseconds to sysTime
	uint32_t SYSCLOCK_HZ = (uint32_t)VARIANT_MCK;
	uint32_t TIMCLK_Hz = SYSCLOCK_HZ / TIMER_PRESCALER_DIV;
	uint32_t NANOSECONDS_PER_TICK = 1000000000 / TIMCLK_Hz;
	
	// Calculate compare values
	uint16_t pulseEnd = pulseStart + pulseLength;
	uint16_t CC0_value = (uint16_t)((float)(1000)*pulseStart / (float)NANOSECONDS_PER_TICK);
	uint16_t CC1_value = (uint16_t)((float)(1000)*pulseEnd / (float)NANOSECONDS_PER_TICK);
	
	// Calculate period value
	timerFrequency = constrain(timerFrequency, 20, 255);
	uint16_t PER_value = (uint16_t)((TIMCLK_Hz / timerFrequency) - 1);

	// Constrain in case of overflow
	CC1_value = constrain(CC1_value, 1, PER_value);
	CC0_value = constrain(CC0_value, 0, CC1_value);

	// Set timer period
	g_LP_vsyncTCC->PER.reg = PER_value;
	while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
	
	// Set capture compare 0
	g_LP_vsyncTCC->CC[0].reg = CC0_value;
	while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

	// Set capture compare 1
	g_LP_vsyncTCC->CC[1].reg = CC1_value;
	while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

	// Enable match / capture events for CC0, CC1
	g_LP_vsyncTCC->EVCTRL.bit.MCEO0 = 1;
	g_LP_vsyncTCC->EVCTRL.bit.MCEO1 = 1;
	
	// Enable the event interrupts
	g_LP_vsyncTCC->INTENSET.bit.MC0 = 1;
	g_LP_vsyncTCC->INTENSET.bit.MC1 = 1;

	if(startNow == true)
	{
		NVIC_ClearPendingIRQ(g_LP_vsyncTCC_handler);
		NVIC_SetPriority(g_LP_vsyncTCC_handler, 1);
		NVIC_EnableIRQ(g_LP_vsyncTCC_handler);
	}
	
	g_LP_vsyncTCC->CTRLA.bit.ENABLE = 1;
	while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

	if(startNow == false)
	{
		g_LP_vsyncTCC->CTRLA.bit.ENABLE = 0;
		while (g_LP_vsyncTCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
		NVIC_ClearPendingIRQ(g_LP_vsyncTCC_handler);
		NVIC_SetPriority(g_LP_vsyncTCC_handler, 1);
		NVIC_EnableIRQ(g_LP_vsyncTCC_handler);
	}
	
	#else // NOT SAD21
	
	#warning "LP8545 vsync functions only support Arduino Zero"
	
	#endif
}

void LP8545_ :: startEasingTimer(uint8_t timerFrequency)
{
	#ifdef _VARIANT_ARDUINO_ZERO_

	uint16_t GCLK_CLKCTRL_IDx;
	
	if(g_LP_easingTC == TC3)
	{
		GCLK_CLKCTRL_IDx = GCLK_CLKCTRL_ID(GCM_TCC2_TC3);
	}
	else if((g_LP_easingTC == TC4) || (g_LP_easingTC == TC5))
	{
		GCLK_CLKCTRL_IDx = GCLK_CLKCTRL_ID(GCM_TC4_TC5);
	}

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDx);
	while (GCLK->STATUS.bit.SYNCBUSY == 1);

	g_LP_easingTC->COUNT16.INTENCLR.bit.MC0 = 1;
	//TCfade->COUNT16.INTENCLR.bit.MC1 = 1;

	g_LP_easingTC->COUNT16.CTRLA.bit.ENABLE = 0;

	// Set prescaler to 256
	uint16_t TIMER_PRESCALER_DIV = 256;
	g_LP_easingTC->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;
	while (g_LP_easingTC->COUNT16.STATUS.bit.SYNCBUSY);

	// Go from microseconds to sysTime
	uint32_t SYSCLOCK_HZ = (uint32_t)VARIANT_MCK;
	uint32_t TIMCLK_Hz = SYSCLOCK_HZ / TIMER_PRESCALER_DIV;
	
	// Calculate period value
	uint16_t CC0_value = (uint16_t)((TIMCLK_Hz / timerFrequency) - 1);
	
	// Set capture compare 0 (as period)
	g_LP_easingTC->COUNT16.CC[0].reg = CC0_value;
	while (g_LP_easingTC->COUNT16.STATUS.bit.SYNCBUSY);

	// Set capture compare 1
	//TCCvsync->COUNT16.CC[1].reg = CC1_value;
	//while (TCfade->COUNT16.STATUS.bit.SYNCBUSY);

	// Enable match / capture events for CC0, CC1
	g_LP_easingTC->COUNT16.EVCTRL.bit.MCEO0 = 1;
	//TCCvsync->EVCTRL.bit.MCEO1 = 1;
	
	// Enable the event interrupts
	g_LP_easingTC->COUNT16.INTENSET.bit.MC0 = 1;
	//TCfade->COUNT8.INTENSET.bit.MC1 = 1;

	NVIC_ClearPendingIRQ(g_LP_easingTC_handler);
	NVIC_SetPriority(g_LP_easingTC_handler, 1);
	NVIC_EnableIRQ(g_LP_easingTC_handler);
	
	g_LP_easingTC->COUNT16.CTRLA.bit.ENABLE = 1;
	while (g_LP_easingTC->COUNT16.STATUS.bit.SYNCBUSY);
	
	g_LP_easingTC_isRunning = true;
	
	#endif
}

void LP8545_ :: setRegister(uint8_t reg, uint8_t dataByte)
{
	uint8_t pData[2];
	
	pData[0] = reg;
	pData[1] = dataByte;
	
	i2c_tx(LP_I2C_ADDRESS, &pData[0], countBytes(pData));
}

void LP8545_ :: readRegister(uint8_t reg, uint8_t * readByte)
{
	uint8_t pData[1];

	pData[0] = reg;
	
	i2c_tx(LP_I2C_ADDRESS, &pData[0], countBytes(pData));
	
	i2c_rx(LP_I2C_ADDRESS, readByte, countBytes(pData));
}

void LP8545_ :: readRegisterDebug(uint8_t reg)
{
	uint8_t pData[1];
	uint8_t readByte;

	pData[0] = reg;
	
	i2c_tx(LP_I2C_ADDRESS, &pData[0], countBytes(pData));

	i2c_rx(LP_I2C_ADDRESS, &readByte, countBytes(pData));
	
	#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
	#define BYTE_TO_BINARY(byte)  \
	(byte & 0x80 ? '1' : '0'), \
	(byte & 0x40 ? '1' : '0'), \
	(byte & 0x20 ? '1' : '0'), \
	(byte & 0x10 ? '1' : '0'), \
	(byte & 0x08 ? '1' : '0'), \
	(byte & 0x04 ? '1' : '0'), \
	(byte & 0x02 ? '1' : '0'), \
	(byte & 0x01 ? '1' : '0')
	
	SerialUSB.printf("0x%02x: 0b%c%c%c%c%c%c%c%c, %d \n", reg, BYTE_TO_BINARY(readByte), readByte);
}

void LP8545_ :: i2c_tx(uint8_t addr, uint8_t* pData, uint8_t len)
{
	Wire.beginTransmission(addr);
	
	for(uint8_t i = 0; i < len; i++){
		uint8_t databyte = (uint8_t)(pData[i]);
		volatile uint8_t success = Wire.write(databyte);
	}
	
	uint8_t result = Wire.endTransmission();
}

void LP8545_ :: i2c_rx(uint8_t addr, uint8_t *pData, uint8_t len)
{
	Wire.requestFrom(addr, len);
	
	uint8_t ack = false;
	
	uint8_t i = 0;
	
	uint8_t *readPtr = pData;

	/// @todo: Might write over memory if pData too small...
	while(Wire.available())
	{
		*readPtr = Wire.read();
		readPtr++;
		ack = true;
	}
}

// ----------------------------------------------------------------------------------
// Configuration
void LP8545_ :: setFactoryDefaults(void)
{
	// Eeprom 1
	setBoostFreq(boostFreqConfig::defaultValue);
	setLedFaultDetection(ledFaultsConfig::defaultValue);
	setTempLimit(tempLimitConfig::defaultValue);
	setSlope(slopeConfig::defaultValue, advancedSlopeConfig::defaultValue);
	
	// Eeprom 2
	setAdaptiveSpeed(adaptiveSpeedConfig1::defaultValue, adaptiveSpeedConfig2::defaultValue);
	setFetEnable(enExternalFetConfig::defaultValue);
	setAdaptiveMode(enAdaptiveModeConfig::defaultValue);
	setBoostOnOff(enableBoostConfig::defaultValue);
	setBoostCurrent(boostMaxCurrentConfig::defaultValue);

	// Eeprom 3
	setUVLO(LP_uvloConfig_t::defaultValue);
	setPhaseShiftPWM(LP_enPhaseShiftPwmConfig_t::defaultValue);
	setPWMFreq(0);
	
	// Eeprom 4
	setPwmResolution(LP_pwmResolutionConfig_t::defaultValue);
	setIresEnable(LP_enIresConfig_t::defaultValue);
	setLedFaultThreshold (LP_ledFaultThresholdConfig_t::defaultValue);
	setLedFaultHeadrooom (LP_ledFaultHeadroomConfig_t::defaultValue);
	
	// Eeprom 5
	setVsync(LP_enVsyncConfig_t::defaultValue);
	setDither(LP_ditherConfig_t::defaultValue);
	setVboost(0);
	
	// Eeprom 6
	setPll(0);
	setFresEnabled(LP_enFresConfig_t::defaultValue);
	setHysteresisResolution(LP_hysteresisConfig_t::defaultValue);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_BRIGHT_CTL
void LP8545_ :: setBrightCtl(uint8_t brightness)
{
	uint8_t setting_bits = brightness;
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_BRIGHT_CTL, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_BRT) | (LP_BITMASK_BRT & setting_bits);
	setRegister(LP_REGISTER_BRIGHT_CTL, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_DEVICE_CTL
void LP8545_ :: setBrtMode(LP_brtModeConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator){
		case brtModeConfig::brtModePwmInputDutyCycleControl:
		case brtModeConfig::defaultValue:
		{
			setting_bits = 0b00000000;
			break;
		}
		case brtModeConfig::brtModeBrightnessRegister:
		default:
		{
			setting_bits = 0b00000100;
			break;
		}
		case brtModeConfig::brtModeDirectControl:
		{
			setting_bits = 0b00000110;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_DEVICE_CTL, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_BRT_MODE) | (LP_BITMASK_BRT_MODE & setting_bits);
	setRegister(LP_REGISTER_DEVICE_CTL, newRegister);
}

void LP8545_ :: setBlCtl(LP_blCtlConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator){
		case blCtlConfig::blCtlOff:
		case blCtlConfig::defaultValue:
		{
			setting_bits = 0b00000000;
			break;			
		}
		case blCtlConfig::blCtlOn:
		default:
		{
			setting_bits = 0b00000001;
			break;			
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_DEVICE_CTL, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_BL_CTL) | (LP_BITMASK_BL_CTL & setting_bits);
	setRegister(LP_REGISTER_DEVICE_CTL, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_DIRECT_CTL
void LP8545_ :: setDirectCtl(bool out1, bool out2, bool out3, bool out4, bool out5, bool out6)
{
	uint8_t setting_bits = 0;
	
	if(out1)
	{
		setting_bits = setting_bits | LP_BITMASK_OUT_1;
	}
	
	if(out2)
	{
		setting_bits = setting_bits | LP_BITMASK_OUT_2;
	}
	
	if(out3)
	{
		setting_bits = setting_bits | LP_BITMASK_OUT_3;
	}
	
	if(out4)
	{
		setting_bits = setting_bits | LP_BITMASK_OUT_4;
	}
	
	if(out5)
	{
		setting_bits = setting_bits | LP_BITMASK_OUT_5;
	}
	
	if(out6)
	{
		setting_bits = setting_bits | LP_BITMASK_OUT_6;
	}
	
	uint8_t newRegister = setting_bits & 0b00111111;
	setRegister(LP_REGISTER_DIRECT_CTL, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_CTL
void LP8545_ :: setEepromCtl(bool EE_init, bool EE_prog, bool EE_read)
{
	uint8_t EE_INIT_BIT = 0;
	uint8_t EE_PROG_BIT = 0;
	uint8_t EE_READ_BIT = 0;
	
	if(EE_init)
	{
		EE_INIT_BIT	= LP_BITMASK_EE_INIT;
	}
	
	if(EE_prog)
	{
		EE_PROG_BIT = LP_BITMASK_EE_PROG;
	}
	
	if(EE_read)
	{
		EE_READ_BIT = LP_BITMASK_EE_READ;
	}
	
	uint8_t newRegister = EE_INIT_BIT || EE_PROG_BIT || EE_READ_BIT;
	setRegister(LP_REGISTER_EEPROM_CTL, newRegister);
}

void LP8545_ :: saveToNVM()
{
	setEepromCtl(true, false, false);
	setEepromCtl(false, true, false);
	delay(200);
	setEepromCtl(false, false, false);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_0
void LP8545_ :: setCurrentMicroamps(uint16_t microAmps)
{
	uint8_t newRegister = map(microAmps, 0, 30000, 0, 255);
	
	setRegister(LP_REGISTER_EEPROM_0, LP_BITMASK_CURRENT & newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_1
void LP8545_ :: setBoostFreq(LP_boostFreqConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case boostFreqConfig::boostFreq115kHz:
		{
			setting_bits = 0b00000000;
			break;
		}
		case boostFreqConfig::boostFreq312kHz:
		case boostFreqConfig::defaultValue:
		default:
		{
			setting_bits = 0b01000000;
			break;
		}
		case boostFreqConfig::boostFreq625kHz:
		{
			setting_bits = 0b10000000;
			break;
		}
		case boostFreqConfig::boostFreq1250kHz:
		{
			setting_bits = 0b11000000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_1, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_BOOST_FREQ) | (LP_BITMASK_BOOST_FREQ & setting_bits);
	setRegister(LP_REGISTER_EEPROM_1, newRegister);
}

void LP8545_ :: setLedFaultDetection(LP_ledFaultsConfig_t enumerator){
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case ledFaultsConfig::ledFaultsDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case ledFaultsConfig::ledFaultsEnabled:
		case ledFaultsConfig::defaultValue:
		default:
		{
			setting_bits = 0b00100000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_1, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_LED_FAULT) | (LP_BITMASK_EN_LED_FAULT & setting_bits);
	setRegister(LP_REGISTER_EEPROM_1, newRegister);
}

void LP8545_ :: setTempLimit(LP_tempLimitConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case tempLimitConfig::tempLimitDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case tempLimitConfig::tempLimit110C:
		{
			setting_bits = 0b00001000;
			break;
		}
		case tempLimitConfig::tempLimit120C:
		case tempLimitConfig::defaultValue:
		default:
		{
			setting_bits = 0b00010000;
			break;
		}
		case tempLimitConfig::tempLimit130C:
		{
			setting_bits = 0b00011000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_1, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_TEMP_LIM) | (LP_BITMASK_TEMP_LIM & setting_bits);
	setRegister(LP_REGISTER_EEPROM_1, newRegister);
}

void LP8545_ :: setSlope(LP_slopeConfig_t enumerator0, LP_advancedSlopeConfig_t enumerator1)
{
	uint8_t setting_bits;
	
	switch(enumerator0){
		case slopeConfig::slopeDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case slopeConfig::slope50ms:
		case slopeConfig::defaultValue:
		default:
		{
			setting_bits = 0b00000001;
			break;
		}
		case slopeConfig::slope75ms:
		{
			setting_bits = 0b00000010;
			break;
		}
		case slopeConfig::slope100ms:
		{
			setting_bits = 0b00000011;
			break;
		}
		case slopeConfig::slope150ms:
		{
			setting_bits = 0b00000100;
			break;
		}
		case slopeConfig::slope200ms:
		{
			setting_bits = 0b00000101;
			break;
		}
		case slopeConfig::slope300ms:
		{
			setting_bits = 0b00000110;
			break;
		}
		case slopeConfig::slope500ms:
		{
			setting_bits = 0b00000111;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_1, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_SLOPE) | (LP_BITMASK_SLOPE & setting_bits);
	setRegister(LP_REGISTER_EEPROM_1, newRegister);
	
	// CheckOn defaultval
	
	switch(enumerator1)
	{
		case advancedSlopeConfig::advancedSlopeDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case advancedSlopeConfig::advancedSlopeEnabled:
		case advancedSlopeConfig::defaultValue:
		default:
		{
			setting_bits = 0b00100000;
			break;
		}
	}
	
	readRegister(LP_REGISTER_EEPROM_2, &oldRegister);
	
	newRegister = (oldRegister & ~LP_BITMASK_ADV_SLOPE) | (LP_BITMASK_ADV_SLOPE & setting_bits);
	setRegister(LP_REGISTER_EEPROM_2, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_2
void LP8545_ :: setAdaptiveSpeed(LP_adaptiveSpeedConfig_1_t enumerator0, LP_adaptiveSpeedConfig_2_t enumerator1)
{
	uint8_t setting_bits_0;
	uint8_t setting_bits_1;
	
	switch(enumerator0)
	{
		case adaptiveSpeedConfig1::adaptiveSpeedNormalMode:
		{
			setting_bits_0 = 0b00000000;
			break;
		}
		case adaptiveSpeedConfig1::adaptiveSpeedLightLoads:
		case adaptiveSpeedConfig1::defaultValue:
		default:
		{
			setting_bits_0 = 0b10000000;
			break;
		}
	}

	switch(enumerator1)
	{
		case adaptiveSpeedConfig2::adaptiveSpeedOncePerCycle:
		{
			setting_bits_1 = 0b00000000;
			break;
		}
		case adaptiveSpeedConfig2::adaptiveSpeedOncePer16Cycle:
		case adaptiveSpeedConfig2::defaultValue:
		default:
		{
			setting_bits_1 = 0b01000000;
			break;
		}
	}
	
	uint8_t result = setting_bits_0 | setting_bits_1;
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_2, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_ADAPTIVE_SPEED) | (LP_BITMASK_ADAPTIVE_SPEED & result);
	setRegister(LP_REGISTER_EEPROM_2, newRegister);
}

void LP8545_ :: setFetEnable(LP_enExternalFetConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case enExternalFetConfig::externalFetDisabled:
		case enExternalFetConfig::defaultValue:
		default:
		{
			setting_bits = 0b00000000;
			break;
		}
		case enExternalFetConfig::externalFetEnabled:
		{
			setting_bits = 0b00010000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_2, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_EXT_FET) | (LP_BITMASK_EN_EXT_FET & setting_bits);
	setRegister(LP_REGISTER_EEPROM_2, newRegister);
}

void LP8545_ :: setAdaptiveMode(LP_enAdaptiveModeConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case enAdaptiveModeConfig::adaptiveModeDisable:
		{
			setting_bits = 0b00000000;
			break;
		}
		case enAdaptiveModeConfig::adaptiveModeEnabled:
		case enAdaptiveModeConfig::defaultValue:
		default:
		{
			setting_bits = 0b00001000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_2, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_ADAPT) | (LP_BITMASK_EN_ADAPT & setting_bits);
	setRegister(LP_REGISTER_EEPROM_2, newRegister);
}

void LP8545_ :: setBoostOnOff(LP_enableBoostConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case enableBoostConfig::boostDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case enableBoostConfig::boostEnabled:
		case enableBoostConfig::defaultValue:
		default:
		{
			setting_bits = 0b00000100;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_2, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_BOOST) | (LP_BITMASK_EN_BOOST & setting_bits);
	setRegister(LP_REGISTER_EEPROM_2, newRegister);
}

void LP8545_ :: setBoostCurrent(LP_boostMaxCurrentConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case boostMaxCurrentConfig::maxBoostCurrent900mA:
		{
			setting_bits = 0b00000000;
			break;
		}
		case boostMaxCurrentConfig::maxBoostCurrent1400mA:
		{
			setting_bits = 0b00000001;
			break;
		}
		case boostMaxCurrentConfig::maxBoostCurrent2000mA:
		{
			setting_bits = 0b00000010;
			break;
		}
		case boostMaxCurrentConfig::maxBoostCurrent2500mA:
		case boostMaxCurrentConfig::defaultValue:
		default:
		{
			setting_bits = 0b00000011;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_2, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_BOOST_MAX) | (LP_BITMASK_BOOST_MAX & setting_bits);
	setRegister(LP_REGISTER_EEPROM_2, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_3
void LP8545_ :: setUVLO(LP_uvloConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case uvloConfig::uvloDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case uvloConfig::uvlo2v7:
		case uvloConfig::defaultValue:
		default:
		{
			setting_bits = 0b01000000;
			break;
		}
		case uvloConfig::uvlo6v:
		{
			setting_bits = 0b10000000;
			break;
		}
		case uvloConfig::uvlo9v:
		{
			setting_bits = 0b11000000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_3, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_UVLO) | (LP_BITMASK_UVLO & setting_bits);
	setRegister(LP_REGISTER_EEPROM_3, newRegister);
}

void LP8545_ :: setPhaseShiftPWM(LP_enPhaseShiftPwmConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case enPhaseShiftPwmConfig::phaseShiftPWMDisable:
		{
			setting_bits = 0b00000000;
			break;
		}
		case enPhaseShiftPwmConfig::phaseShiftPWMEnable:
		case enPhaseShiftPwmConfig::defaultValue:
		default:
		{
			setting_bits = 0b00100000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_3, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_PSPWM) | (LP_BITMASK_EN_PSPWM & setting_bits);
	setRegister(LP_REGISTER_EEPROM_3, newRegister);
}

void LP8545_ :: setPWMFreq(uint16_t pwmFreq)
{
	uint8_t setting_bits;
	
	if(pwmFreq == 0)
	{
		//Default
		setting_bits = 0b11011;
		goto LD_pwmDefault;
	}
	
	pwmFreq = constrain(pwmFreq, 600, 19200);
	
	setting_bits = map(pwmFreq, 600, 19200, 0, 0b11111);
	
	LD_pwmDefault:
	
	uint8_t oldRegister;
	
	readRegister(LP_REGISTER_EEPROM_3, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_PWM_FREQ) | (LP_BITMASK_PWM_FREQ & setting_bits);
	
	setRegister(LP_REGISTER_EEPROM_3, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_4
void LP8545_ :: setPwmResolution(LP_pwmResolutionConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case pwmResolutionConfig::pwmResolutionLOW:
		case pwmResolutionConfig::defaultValue:
		default:
		{
			setting_bits = 0b00000000;
			break;
		}
		case pwmResolutionConfig::pwmResolutionMEDIUM:
		{
			setting_bits = 0b01000000;
			break;
		}
		case pwmResolutionConfig::pwmResolutionHIGH:
		{
			setting_bits = 0b10000000;
			break;
		}
		case pwmResolutionConfig::pwmResolutionTURNED_PAST_11:
		{
			setting_bits = 0b11000000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_4, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_PWM_RESOLUTION) | (LP_BITMASK_PWM_RESOLUTION & setting_bits);
	setRegister(LP_REGISTER_EEPROM_4, newRegister);
}

void LP8545_ :: setIresEnable(LP_enIresConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator){
		case enIresConfig::iresDisabled:
		case enIresConfig::defaultValue:
		{
			setting_bits = 0b00000000;
			break;
		}
		case enIresConfig::iresEnabled:
		default:
		{
			setting_bits = 0b00100000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_4, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_IRES) | (LP_BITMASK_EN_IRES & setting_bits);
	setRegister(LP_REGISTER_EEPROM_4, newRegister);
}

void LP8545_ :: setLedFaultThreshold (LP_ledFaultThresholdConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator){
		case ledFaultThresholdConfig::ledFaultThreshold2v3:
		{
			setting_bits = 0b00000000;
			break;
		}
		case ledFaultThresholdConfig::ledFaultThreshold3v3:
		case ledFaultThresholdConfig::defaultValue:
		default:
		{
			setting_bits = 0b00001000;
			break;
		}
		case ledFaultThresholdConfig::ledFaultThreshold4v3:
		{
			setting_bits = 0b00010000;
			break;
		}
		case ledFaultThresholdConfig::ledFaultThreshold5v3:
		{
			setting_bits = 0b00011000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_4, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_LED_FAULT_THR) | (LP_BITMASK_LED_FAULT_THR & setting_bits);
	setRegister(LP_REGISTER_EEPROM_4, newRegister);
}

void LP8545_ :: setLedFaultHeadrooom (LP_ledFaultHeadroomConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case ledFaultHeadroomConfig::ledHeadroomFault125mV:
		case ledFaultHeadroomConfig::defaultValue:
		default:
		{
			setting_bits = 0b00000000;
			break;
		}
		case ledFaultHeadroomConfig::ledHeadroomFault250mV:
		{
			setting_bits = 0b00000001;
			break;
		}
		case ledFaultHeadroomConfig::ledHeadroomFault375mV:
		{
			setting_bits = 0b00000010;
			break;
		}
		case ledFaultHeadroomConfig::ledHeadroomFault500mV:
		{
			setting_bits = 0b00000011;
			break;
		}
		case ledFaultHeadroomConfig::ledHeadroomFault625mV:
		{
			setting_bits = 0b00000100;
			break;
		}
		case ledFaultHeadroomConfig::ledHeadroomFault750mV:
		{
			setting_bits = 0b00000101;
			break;
		}
		case ledFaultHeadroomConfig::ledHeadroomFault875mV:
		{
			setting_bits = 0b00000110;
			break;
		}
		case ledFaultHeadroomConfig::ledHeadroomFault1000mV:
		{
			setting_bits = 0b00000111;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_4, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_DRV_HEADR) | (LP_BITMASK_DRV_HEADR & setting_bits);
	setRegister(LP_REGISTER_EEPROM_4, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_5
void LP8545_ :: setVsync(LP_enVsyncConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case enVsyncConfig::vsyncDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case enVsyncConfig::vsyncEnabled:
		case enVsyncConfig::defaultValue:
		default:
		{
			setting_bits = 0b10000000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_5, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_VSYNC) | (LP_BITMASK_EN_VSYNC & setting_bits);
	setRegister(LP_REGISTER_EEPROM_5, newRegister);
}

void LP8545_ :: setDither(LP_ditherConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case ditherConfig::ditherDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case ditherConfig::dither1bit:
		{
			setting_bits = 0b00100000;
			break;
		}
		case ditherConfig::dither2bit:
		case ditherConfig::defaultValue:
		default:
		{
			setting_bits = 0b01000000;
			break;
		}
		case ditherConfig::dither3bit:
		{
			setting_bits = 0b01100000;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_5, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_DITHER) | (LP_BITMASK_DITHER & setting_bits);
	setRegister(LP_REGISTER_EEPROM_5, newRegister);
}

void LP8545_ :: setVboost(uint8_t vboost)
{
	
	uint8_t setting_bits;
	
	if(vboost == 0)
	{
		// Default
		setting_bits = 0b01111;
		goto LD_vboostDefault;
	}
	
	vboost = constrain(vboost, 10, 40);
	
	setting_bits = map(vboost, 10, 40, 0, 0b11111);
	
	LD_vboostDefault:
	
	uint8_t oldRegister;
	
	readRegister(LP_REGISTER_EEPROM_5, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_VBOOST) | (LP_BITMASK_VBOOST & setting_bits);
	
	setRegister(LP_REGISTER_EEPROM_5, newRegister);
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_EEPROM_6, LP_REGISTER_EEPROM_7
void LP8545_ :: setPll(uint16_t pll)
{
	if(pll == 0)
	{
		// Default
		pll = 3205;
	};
	
	uint8_t setting_bits_high = pll >> 5;
	uint8_t setting_bits_low  = (pll << 3) & 255;

	uint8_t oldRegister_high;
	uint8_t oldRegister_low;

	readRegister(LP_REGISTER_EEPROM_6, &oldRegister_high);
	readRegister(LP_REGISTER_EEPROM_7, &oldRegister_low);

	uint8_t newRegister_high = (oldRegister_high & ~LP_BITMASK_PLL_HIGHBITS) | (LP_BITMASK_PLL_HIGHBITS & setting_bits_high);
	uint8_t newRegister_low = (oldRegister_low & ~LP_BITMASK_PLL_LOWBITS) | (LP_BITMASK_PLL_LOWBITS & setting_bits_low);

	setRegister(LP_REGISTER_EEPROM_6, newRegister_high);
	setRegister(LP_REGISTER_EEPROM_7, newRegister_low);
};

void LP8545_ :: setFresEnabled(LP_enFresConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case enFresConfig::fresDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case enFresConfig::fresEnabled:
		case enFresConfig::defaultValue:
		default:
		{
			setting_bits = 0b00000100;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_7, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_EN_F_RES) | (LP_BITMASK_EN_F_RES & setting_bits);
	setRegister(LP_REGISTER_EEPROM_7, newRegister);
}

void LP8545_ :: setHysteresisResolution(LP_hysteresisConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator)
	{
		case hysteresisConfig::hysteresisDisabled:
		{
			setting_bits = 0b00000000;
			break;
		}
		case hysteresisConfig::hysteresis11bit:
		default:
		{
			setting_bits = 0b00000001;
			break;
		}
		case hysteresisConfig::hysteresis10bit:
		{
			setting_bits = 0b00000010;
			break;
		}
		case hysteresisConfig::hysteresis8bit:
		{
			setting_bits = 0b00000011;
			break;
		}
	}
	
	uint8_t oldRegister;
	readRegister(LP_REGISTER_EEPROM_7, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_HYSTERESIS) | (LP_BITMASK_HYSTERESIS & setting_bits);
	setRegister(LP_REGISTER_EEPROM_7, newRegister);
}

// ----------------------------------------------------------------------------------
// Misc
uint32_t LP8545_ :: mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
	if (from == to)
	{
		return value;
	}
	if (from > to)
	{
		return value >> (from-to);
	}
	return value << (to-from);
}

float LP8545_ :: easingFunction(float x, LP_easingFunctionType_t functionType)
{
	switch(functionType)
	{
		// Modeled after the line y = x
		case easingFunctionSetup::linear:
		{
			return x;
		}
		
		// Modeled after the piecewise quadratic
		// y = (1/2)((2x)^2)             ; [0, 0.5)
		// y = -(1/2)((2x-1)*(2x-3) - 1) ; [0.5, 1
		case easingFunctionSetup::quadratic:
		{
			if(x < 0.5)
			{
				return 2 * x * x;
			}
			else
			{
				return (-2 * x * x) + (4 * x) - 1;
			}
		}
		
		// Modeled after the piecewise cubic
		// y = (1/2)((2x)^3)       ; [0, 0.5)
		// y = (1/2)((2x-2)^3 + 2) ; [0.5, 1]
		case easingFunctionSetup::cubic:
		{
			if(x < 0.5)
			{
				return 4 * x * x * x;
			}
			else
			{
				float f = ((2 * x) - 2);
				return 0.5 * f * f * f + 1;
			}
		}
		
		// Modeled after the piecewise quartic
		// y = (1/2)((2x)^4)        ; [0, 0.5)
		// y = -(1/2)((2x-2)^4 - 2) ; [0.5, 1]
		case easingFunctionSetup::quartic:
		{
			if(x < 0.5)
			{
				return 8 * x * x * x * x;
			}
			else
			{
				float f = (x - 1);
				return -8 * f * f * f * f + 1;
			}
		}
		
		// Modeled after the piecewise quintic
		// y = (1/2)((2x)^5)       ; [0, 0.5)
		// y = (1/2)((2x-2)^5 + 2) ; [0.5, 1]
		case easingFunctionSetup::quintic:
		{
			if(x < 0.5)
			{
				return 16 * x * x * x * x * x;
			}
			else
			{
				float f = ((2 * x) - 2);
				return  0.5 * f * f * f * f * f + 1;
			}
		}
		
		// Modeled after half sine wave
		case easingFunctionSetup::sinusoid:
		{
			return 0.5 * (1 - cos(x * M_PI));
		}
		
		// Modeled after the piecewise circular function
		// y = (1/2)(1 - sqrt(1 - 4x^2))           ; [0, 0.5)
		// y = (1/2)(sqrt(-(2x - 3)*(2x - 1)) + 1) ; [0.5, 1]
		case easingFunctionSetup::circular:
		{
			if(x < 0.5)
			{
				return 0.5 * (1 - sqrt(1 - 4 * (x * x)));
			}
			else
			{
				return 0.5 * (sqrt(-((2 * x) - 3) * ((2 * x) - 1)) + 1);
			}
		}
		
		// Modeled after the piecewise exponential
		// y = (1/2)2^(10(2x - 1))         ; [0,0.5)
		// y = -(1/2)*2^(-10(2x - 1))) + 1 ; [0.5,1]
		case easingFunctionSetup::exponential:
		{
			if(x == 0.0 || x == 1.0) return x;
			
			if(x < 0.5)
			{
				return 0.5 * pow(2, (20 * x) - 10);
			}
			else
			{
				return -0.5 * pow(2, (-20 * x) + 10) + 1;
			}
		}
		
		case easingFunctionSetup::elastic:
		{
			if(x < 0.5)
			{
				return 0.5 * sin(13 * M_PI_2 * (2 * x)) * pow(2, 10 * ((2 * x) - 1));
			}
			else
			{
				return 0.5 * (sin(-13 * M_PI_2 * ((2 * x - 1) + 1)) * pow(2, -10 * (2 * x - 1)) + 2);
			}
		}
	}
}

LP8545_ LP8545;