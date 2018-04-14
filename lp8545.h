/**
* @file lp8545.h
*
* @brief LP8545 Driver _H
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
 
#ifndef _LP8545_H_
	#define _LP8545_H_

	#include <Arduino.h>

	//#############################################################################
	// Macros
	//-----------------------------------------------------------------------------
	// Device address
	#define LP_I2C_ADDRESS					0x2C

	// ----------------------------------------------------------------------------------
	// Register IDs
	#define LP_REGISTER_BRIGHT_CTL			0x00
	#define LP_REGISTER_DEVICE_CTL			0x01
	#define LP_REGISTER_FAULT				0x02
	#define LP_REGISTER_ID					0x03
	#define LP_REGISTER_DIRECT_CTL			0x04
	#define LP_REGISTER_TEMP_MSB			0x05
	#define LP_REGISTER_TEMP_LSB			0x06	
	#define LP_REGISTER_EEPROM_CTL			0x72
	#define LP_REGISTER_EEPROM_0			0xA0
	#define LP_REGISTER_EEPROM_1			0xA1
	#define LP_REGISTER_EEPROM_2			0xA2
	#define LP_REGISTER_EEPROM_3			0xA3
	#define LP_REGISTER_EEPROM_4			0xA4
	#define LP_REGISTER_EEPROM_5			0xA5
	#define LP_REGISTER_EEPROM_6			0xA6
	#define LP_REGISTER_EEPROM_7			0xA7

	// ----------------------------------------------------------------------------------
	// Brightness CTL
	#define LP_BITMASK_BRT					0b11111111

	// Device CTL
	#define LP_BITMASK_BRT_MODE				0b00000110
	#define LP_BITMASK_BL_CTL				0b00000001

	// Fault
	#define LP_BITMASK_FAULT_OPEN			0b10000000
	#define LP_BITMASK_FAULT_SHORT			0b01000000
	#define LP_BITMASK_FAULT_2CHAN			0b00100000
	#define LP_BITMASK_FAULT_1CHAN			0b00010000
	#define LP_BITMASK_FAULT_BL_FAULT		0b00001000
	#define LP_BITMASK_FAULT_OVERCURENT		0b00000100
	#define LP_BITMASK_FAULT_TSD			0b00000010
	#define LP_BITMASK_FAULT_UVLO			0b00000001

	// ID		
	#define LP_BITMASK_PANEL				0b10000000
	#define LB_BITMASK_MFG					0b01111000
	#define LB_BITMASK_REV					0b00000111

	// Direct Control
	#define LP_BITMASK_OUT_6				0b00100000
	#define LP_BITMASK_OUT_5				0b00010000
	#define LP_BITMASK_OUT_4				0b00001000
	#define LP_BITMASK_OUT_3				0b00000100
	#define LP_BITMASK_OUT_2				0b00000010
	#define LP_BITMASK_OUT_1				0b00000001

	// Temp	MSB, LSB	
	#define LP_BITMASK_TEMP_HIBITS			0b11111111
	#define LP_BITMASK_TEMP_LOWBITS			0b11000000

	// Eeprom CTL		
	#define LP_BITMASK_EE_READY				0b10000000
	#define LP_BITMASK_EE_INIT				0b00000100
	#define LP_BITMASK_EE_PROG				0b00000010
	#define LP_BITMASK_EE_READ				0b00000001

	// Eeprom 0		
	#define LP_BITMASK_CURRENT				0b11111111
		
	// Eeprom 1		
	#define LP_BITMASK_BOOST_FREQ			0b11000000
	#define LP_BITMASK_EN_LED_FAULT			0b00100000
	#define LP_BITMASK_TEMP_LIM				0b00011000
	#define LP_BITMASK_SLOPE				0b00000111

	// Eeprom 2		
	#define LP_BITMASK_ADAPTIVE_SPEED		0b11000000
	#define LP_BITMASK_ADV_SLOPE			0b00100000
	#define LP_BITMASK_EN_EXT_FET			0b00010000
	#define LP_BITMASK_EN_ADAPT				0b00001000
	#define LP_BITMASK_EN_BOOST				0b00000100
	#define LP_BITMASK_BOOST_MAX			0b00000011
	
	// Eeprom 3		
	#define LP_BITMASK_UVLO					0b11000000
	#define LP_BITMASK_EN_PSPWM				0b00100000
	#define LP_BITMASK_PWM_FREQ				0b00011111

	// Eeprom 4		
	#define LP_BITMASK_PWM_RESOLUTION		0b11000000
	#define LP_BITMASK_EN_IRES				0b00100000
	#define LP_BITMASK_LED_FAULT_THR		0b00011000
	#define LP_BITMASK_DRV_HEADR			0b00000111

	// Eeprom 5
	#define LP_BITMASK_EN_VSYNC				0b10000000
	#define LP_BITMASK_DITHER				0b01100000
	#define LP_BITMASK_VBOOST				0b00011111

	// Eeprom 6, 7
	#define LP_BITMASK_PLL_HIGHBITS			0b11111111
	#define LP_BITMASK_PLL_LOWBITS			0b11111000
	#define LP_BITMASK_EN_F_RES				0b00000100
	#define LP_BITMASK_HYSTERESIS			0b00000011

	// CIE1931 correction table
	// Automatically generated

	const uint16_t g_LP_cie_12bit[513] = {
		0, 1, 2, 3, 4, 4, 5, 6, 7, 8,
		9, 10, 11, 12, 12, 13, 14, 15, 16, 17,
		18, 19, 20, 20, 21, 22, 23, 24, 25, 26,
		27, 27, 28, 29, 30, 31, 32, 33, 34, 35,
		35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
		45, 46, 47, 48, 49, 50, 51, 52, 54, 55,
		56, 57, 58, 59, 61, 62, 63, 65, 66, 67,
		69, 70, 71, 73, 74, 76, 77, 78, 80, 81,
		83, 85, 86, 88, 89, 91, 93, 94, 96, 98,
		99, 101, 103, 105, 106, 108, 110, 112, 114, 116,
		118, 120, 122, 124, 126, 128, 130, 132, 134, 136,
		138, 140, 143, 145, 147, 149, 152, 154, 156, 159,
		161, 163, 166, 168, 171, 173, 176, 178, 181, 183,
		186, 189, 191, 194, 197, 200, 202, 205, 208, 211,
		214, 217, 219, 222, 225, 228, 231, 234, 238, 241,
		244, 247, 250, 253, 257, 260, 263, 267, 270, 273,
		277, 280, 284, 287, 291, 294, 298, 301, 305, 309,
		313, 316, 320, 324, 328, 331, 335, 339, 343, 347,
		351, 355, 359, 363, 368, 372, 376, 380, 384, 389,
		393, 397, 402, 406, 411, 415, 420, 424, 429, 433,
		438, 443, 447, 452, 457, 462, 467, 471, 476, 481,
		486, 491, 496, 501, 507, 512, 517, 522, 527, 533,
		538, 543, 549, 554, 560, 565, 571, 576, 582, 588,
		593, 599, 605, 610, 616, 622, 628, 634, 640, 646,
		652, 658, 664, 671, 677, 683, 689, 696, 702, 708,
		715, 721, 728, 734, 741, 748, 754, 761, 768, 775,
		781, 788, 795, 802, 809, 816, 823, 830, 837, 845,
		852, 859, 867, 874, 881, 889, 896, 904, 911, 919,
		927, 934, 942, 950, 958, 966, 973, 981, 989, 997,
		1006, 1014, 1022, 1030, 1038, 1047, 1055, 1063, 1072, 1080,
		1089, 1097, 1106, 1115, 1123, 1132, 1141, 1150, 1159, 1168,
		1177, 1186, 1195, 1204, 1213, 1222, 1232, 1241, 1250, 1260,
		1269, 1279, 1288, 1298, 1307, 1317, 1327, 1337, 1346, 1356,
		1366, 1376, 1386, 1396, 1406, 1417, 1427, 1437, 1447, 1458,
		1468, 1479, 1489, 1500, 1510, 1521, 1532, 1542, 1553, 1564,
		1575, 1586, 1597, 1608, 1619, 1630, 1642, 1653, 1664, 1676,
		1687, 1698, 1710, 1722, 1733, 1745, 1757, 1768, 1780, 1792,
		1804, 1816, 1828, 1840, 1852, 1865, 1877, 1889, 1902, 1914,
		1927, 1939, 1952, 1964, 1977, 1990, 2003, 2015, 2028, 2041,
		2054, 2067, 2081, 2094, 2107, 2120, 2134, 2147, 2161, 2174,
		2188, 2201, 2215, 2229, 2243, 2257, 2270, 2284, 2299, 2313,
		2327, 2341, 2355, 2370, 2384, 2398, 2413, 2428, 2442, 2457,
		2472, 2486, 2501, 2516, 2531, 2546, 2561, 2576, 2592, 2607,
		2622, 2638, 2653, 2669, 2684, 2700, 2716, 2731, 2747, 2763,
		2779, 2795, 2811, 2827, 2843, 2860, 2876, 2892, 2909, 2925,
		2942, 2958, 2975, 2992, 3009, 3026, 3042, 3059, 3077, 3094,
		3111, 3128, 3145, 3163, 3180, 3198, 3215, 3233, 3251, 3268,
		3286, 3304, 3322, 3340, 3358, 3376, 3395, 3413, 3431, 3450,
		3468, 3487, 3505, 3524, 3543, 3562, 3580, 3599, 3618, 3637,
		3657, 3676, 3695, 3714, 3734, 3753, 3773, 3793, 3812, 3832,
		3852, 3872, 3892, 3912, 3932, 3952, 3972, 3992, 4013, 4033,
		4054, 4074, 4095,
	};
	
	const uint8_t g_LP_cie_8bit[513] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 3, 3, 3, 3,
		3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
		3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 5, 5, 5, 5, 5, 5, 5,
		5, 5, 5, 5, 6, 6, 6, 6, 6, 6,
		6, 6, 6, 7, 7, 7, 7, 7, 7, 7,
		7, 7, 8, 8, 8, 8, 8, 8, 8, 8,
		9, 9, 9, 9, 9, 9, 9, 10, 10, 10,
		10, 10, 10, 10, 11, 11, 11, 11, 11, 11,
		12, 12, 12, 12, 12, 12, 13, 13, 13, 13,
		13, 13, 14, 14, 14, 14, 14, 15, 15, 15,
		15, 15, 16, 16, 16, 16, 16, 17, 17, 17,
		17, 17, 18, 18, 18, 18, 19, 19, 19, 19,
		19, 20, 20, 20, 20, 21, 21, 21, 21, 22,
		22, 22, 22, 23, 23, 23, 23, 24, 24, 24,
		24, 25, 25, 25, 26, 26, 26, 26, 27, 27,
		27, 28, 28, 28, 28, 29, 29, 29, 30, 30,
		30, 31, 31, 31, 32, 32, 32, 33, 33, 33,
		33, 34, 34, 35, 35, 35, 36, 36, 36, 37,
		37, 37, 38, 38, 38, 39, 39, 39, 40, 40,
		41, 41, 41, 42, 42, 43, 43, 43, 44, 44,
		45, 45, 45, 46, 46, 47, 47, 47, 48, 48,
		49, 49, 50, 50, 50, 51, 51, 52, 52, 53,
		53, 54, 54, 54, 55, 55, 56, 56, 57, 57,
		58, 58, 59, 59, 60, 60, 61, 61, 62, 62,
		63, 63, 64, 64, 65, 65, 66, 66, 67, 67,
		68, 68, 69, 69, 70, 71, 71, 72, 72, 73,
		73, 74, 74, 75, 76, 76, 77, 77, 78, 78,
		79, 80, 80, 81, 81, 82, 83, 83, 84, 84,
		85, 86, 86, 87, 88, 88, 89, 89, 90, 91,
		91, 92, 93, 93, 94, 95, 95, 96, 97, 97,
		98, 99, 99, 100, 101, 102, 102, 103, 104, 104,
		105, 106, 106, 107, 108, 109, 109, 110, 111, 112,
		112, 113, 114, 115, 115, 116, 117, 118, 118, 119,
		120, 121, 122, 122, 123, 124, 125, 126, 126, 127,
		128, 129, 130, 130, 131, 132, 133, 134, 135, 135,
		136, 137, 138, 139, 140, 141, 141, 142, 143, 144,
		145, 146, 147, 148, 148, 149, 150, 151, 152, 153,
		154, 155, 156, 157, 158, 159, 159, 160, 161, 162,
		163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
		173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
		183, 184, 185, 186, 187, 188, 189, 191, 192, 193,
		194, 195, 196, 197, 198, 199, 200, 201, 202, 204,
		205, 206, 207, 208, 209, 210, 211, 213, 214, 215,
		216, 217, 218, 219, 221, 222, 223, 224, 225, 227,
		228, 229, 230, 231, 233, 234, 235, 236, 237, 239,
		240, 241, 242, 244, 245, 246, 247, 249, 250, 251,
		252, 254, 255
	};

	//#############################################################################
	// Type definitions
	//-----------------------------------------------------------------------------
	typedef enum class pinPwm_timerType
	{
		timerTypeTCC,
		timerTypeTC
	} LP_pinPwm_timerType_t;
	

	//#############################################################################
	// Function prototypes
	//-----------------------------------------------------------------------------
	void VSYNC_IRQ(void);
	

	//#############################################################################
	// Shared variables
	//-----------------------------------------------------------------------------	
	extern uint32_t g_LP_pinId_pwmPin;
	

	//#############################################################################
	// Library class
	//-----------------------------------------------------------------------------
	class LP8545_
	{

		// ----------------------------------------------------------------------------------
		public:
		typedef enum class vsyncModeSetup
		{
			vsyncModeOneshot,
			vsyncModeAstableTriggered,
			vsyncModeAstableAutostart
		} LP_config_vsyncType_t;
		
		typedef enum class easingFunctionSetup
		{
			linear,
			quadratic,
			cubic,
			quartic,
			quintic,
			sinusoid,
			circular,
			exponential,
			elastic
		} LP_easingFunctionType_t;
		
		LP8545_(void);
		uint8_t begin(uint32_t pwmPin, uint32_t vsyncInterruptPin);
		void end(void);
		void setup(uint16_t maxLedCurrentuA, uint8_t ledVoltage, LP_easingFunctionType_t easingFunction, LP_config_vsyncType_t vsyncType, uint8_t vsyncFrequency, uint16_t vsyncStartDelay, uint16_t vsyncPulseLength);
		void loopTask(void);
		void brightness(uint16_t pwmVal, bool shouldFade);
		void powerOn(void);
		void powerOff(void);
		void lightOn(bool shouldFade);
		void lightOff(bool shouldFade);
		
		// ----------------------------------------------------------------------------------
		private:
		
		typedef enum class brtModeConfig
		{
			brtModePwmInputDutyCycleControl,
			brtModeBrightnessRegister,
			brtModeDirectControl,
			defaultValue
		} LP_brtModeConfig_t;	
	
		typedef enum class blCtlConfig
		{
			blCtlOff,
			blCtlOn,
			defaultValue
		} LP_blCtlConfig_t;
			
		// Eeprom 1	
		typedef enum class boostFreqConfig
		{
			boostFreq115kHz,
			boostFreq312kHz,
			boostFreq625kHz,
			boostFreq1250kHz,
			defaultValue	
		} LP_boostFreqConfig_t;

		typedef enum class ledFaultsConfig
		{
			ledFaultsDisabled,
			ledFaultsEnabled,
			defaultValue
		} LP_ledFaultsConfig_t;

		typedef enum class tempLimitConfig
		{
			tempLimitDisabled,
			tempLimit110C,
			tempLimit120C,
			tempLimit130C,
			defaultValue
		} LP_tempLimitConfig_t;

		typedef enum class slopeConfig
		{
			slopeDisabled,
			slope50ms,
			slope75ms,
			slope100ms,
			slope150ms,
			slope200ms,
			slope300ms,
			slope500ms,
			defaultValue
		} LP_slopeConfig_t;

		// Eeprom 2
		typedef enum class adaptiveSpeedConfig1
		{
			adaptiveSpeedNormalMode,
			adaptiveSpeedLightLoads,
			defaultValue
		} LP_adaptiveSpeedConfig_1_t;
	
		typedef enum class adaptiveSpeedConfig2
		{
			adaptiveSpeedOncePerCycle,
			adaptiveSpeedOncePer16Cycle,
			defaultValue
		} LP_adaptiveSpeedConfig_2_t;

		typedef enum class advancedSlopeConfig
		{
			advancedSlopeDisabled,
			advancedSlopeEnabled,
			defaultValue
		} LP_advancedSlopeConfig_t;

		typedef enum class enExternalFetConfig
		{
			externalFetDisabled,
			externalFetEnabled,
			defaultValue
		} LP_enExternalFetConfig_t;

		typedef enum class enAdaptiveModeConfig
		{
			adaptiveModeDisable,
			adaptiveModeEnabled,
			defaultValue
		} LP_enAdaptiveModeConfig_t;

		typedef enum class enableBoostConfig
		{
			boostDisabled,
			boostEnabled,
			defaultValue
		} LP_enableBoostConfig_t;

		typedef enum class boostMaxCurrentConfig
		{
			maxBoostCurrent900mA,
			maxBoostCurrent1400mA,
			maxBoostCurrent2000mA,
			maxBoostCurrent2500mA,
			defaultValue
		} LP_boostMaxCurrentConfig_t;
	
		// Eeprom 3
		typedef enum class uvloConfig
		{
			uvloDisabled,
			uvlo2v7,
			uvlo6v,
			uvlo9v,
			defaultValue
		} LP_uvloConfig_t;	

		typedef enum class enPhaseShiftPwmConfig
		{
			phaseShiftPWMDisable,
			phaseShiftPWMEnable,
			defaultValue
		} LP_enPhaseShiftPwmConfig_t;
	
		// Eeprom 4	
		typedef enum class pwmResolutionConfig
		{
			pwmResolutionLOW,
			pwmResolutionMEDIUM,
			pwmResolutionHIGH,
			pwmResolutionTURNED_PAST_11,
			defaultValue
		} LP_pwmResolutionConfig_t;

		typedef enum class enIresConfig
		{
			iresDisabled,
			iresEnabled,
			defaultValue
		} LP_enIresConfig_t;

		typedef enum class ledFaultThresholdConfig
		{
			ledFaultThreshold2v3,
			ledFaultThreshold3v3,
			ledFaultThreshold4v3,
			ledFaultThreshold5v3,
			defaultValue	
		} LP_ledFaultThresholdConfig_t;

		typedef enum class ledFaultHeadroomConfig
		{
			ledHeadroomFault125mV,
			ledHeadroomFault250mV,
			ledHeadroomFault375mV,
			ledHeadroomFault500mV,
			ledHeadroomFault625mV,
			ledHeadroomFault750mV,
			ledHeadroomFault875mV,
			ledHeadroomFault1000mV,
			defaultValue	
		} LP_ledFaultHeadroomConfig_t;

		// Eeprom 5
		typedef enum class enVsyncConfig
		{
			vsyncDisabled,
			vsyncEnabled,
			defaultValue		
		} LP_enVsyncConfig_t;

		typedef enum class ditherConfig
		{
			ditherDisabled,
			dither1bit,
			dither2bit,
			dither3bit,
			defaultValue
		} LP_ditherConfig_t;

		// Eeprom 6, 7
		typedef enum class enFresConfig
		{
			fresDisabled,
			fresEnabled,
			defaultValue
		} LP_enFresConfig_t;

		typedef enum class hysteresisConfig
		{
			hysteresisDisabled,
			hysteresis11bit,
			hysteresis10bit,
			hysteresis8bit,
			defaultValue		
		} LP_hysteresisConfig_t;
		
		// ----------------------------------------------------------------------------------
		// Hardware Abstraction Layer
		void startVsyncTimer(bool oneShotEnabled, uint8_t timerFrequency, uint16_t pulseStart, uint16_t pulseLength, bool startNow); 	
		void setPwmPinTimer(uint32_t pin, uint32_t value);
		void startEasingTimer(uint8_t timerFrequency);
		void setRegister(uint8_t reg, uint8_t dataByte);
		void readRegister(uint8_t reg, uint8_t * readByte);
		void readRegisterDebug(uint8_t reg);
		void i2c_tx(uint8_t addr, uint8_t * pData, uint8_t len);
		void i2c_rx(uint8_t addr, uint8_t * pData, uint8_t len);
		
		// Config		
		void setFactoryDefaults(void);		
				
		// LP_REGISTER_BRIGHT_CTL
		void setBrightCtl(uint8_t brightnes);

		// LP_REGISTER_DEVICE_CTL
		void setBrtMode(LP_brtModeConfig_t enumerator);
		void setBlCtl(LP_blCtlConfig_t enumerator);
		
		// LP_REGISTER_DIRECT_CTL
		void setDirectCtl(bool out1, bool out2, bool out3, bool out4, bool out5, bool out6);
		
		// LP_REGISTER_EEPROM_CTL
		void setEepromCtl(bool EE_init, bool EE_prog, bool EE_read);
		void saveToNVM(void);

		//Eeprom 0
		void setCurrentMicroamps(uint16_t microAmps);
		
		//Eeprom 1
		void setBoostFreq(LP_boostFreqConfig_t enumerator);
		void setLedFaultDetection(LP_ledFaultsConfig_t enumerator);
		void setTempLimit(LP_tempLimitConfig_t enumerator);
		void setSlope(LP_slopeConfig_t enumerator0, LP_advancedSlopeConfig_t enumerator1);
	
		// Eeprom 2
		void setAdaptiveSpeed(LP_adaptiveSpeedConfig_1_t enumerator0, LP_adaptiveSpeedConfig_2_t enumerator1);
		void setFetEnable(LP_enExternalFetConfig_t enumerator);
		void setAdaptiveMode(LP_enAdaptiveModeConfig_t enumerator);
		void setBoostOnOff(LP_enableBoostConfig_t enumerator);
		void setBoostCurrent(LP_boostMaxCurrentConfig_t enumerator);
		
		// Eeprom 3
		void setUVLO(LP_uvloConfig_t enumerator);
		void setPhaseShiftPWM(LP_enPhaseShiftPwmConfig_t enumerator);
		void setPWMFreq(uint16_t pwmFreq);
		
		// Eeprom 4
		void setPwmResolution(LP_pwmResolutionConfig_t enumerator);
		void setIresEnable(LP_enIresConfig_t enumerator);
		void setLedFaultThreshold (LP_ledFaultThresholdConfig_t enumerator);
		void setLedFaultHeadrooom (LP_ledFaultHeadroomConfig_t enumerator);
		
		// Eeprom 5
		void setVsync(LP_enVsyncConfig_t enumerator);
		void setDither(LP_ditherConfig_t enumerator);
		void setVboost(uint8_t vboost);
		
		// Eeprom 6, 7
		void setPll(uint16_t pll);
		void setFresEnabled (LP_enFresConfig_t enumerator);
		void setHysteresisResolution(LP_hysteresisConfig_t enumerator);
		
		// Misc
		uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);
		float easingFunction(float x, LP_easingFunctionType_t functionType);
	};			
	
	extern LP8545_ LP8545;	
		
#endif
