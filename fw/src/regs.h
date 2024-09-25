/**
  ******************************************************************************
  * @file           : regs.h
  * @brief          : Code to handle registers that do not survive power cycle
  ******************************************************************************
  */
#ifndef __REGS_H__
#define __REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define REG_SIZE 512
#define REG_SIZE8      (REG_SIZE-4)
#define REG_SIZE16     (REG_SIZE8/2)
#define REG_SIZE32     (REG_SIZE16/2)

typedef struct REG_BLOCK {
	 // 512 bytes - 4 per page
    uint8_t start;
	uint8_t seq;
	uint8_t ver;
	uint8_t chksum;
	union {
		uint8_t u8[REG_SIZE8];
		uint16_t u16[REG_SIZE16];
		int16_t s16[REG_SIZE16];
		uint32_t u32[REG_SIZE32];
		int32_t s32[REG_SIZE32];
		float f32[REG_SIZE32];
	};
} REG_BLOCK;

extern volatile REG_BLOCK Regs;
	
enum SYS_STATUS {
  RegFirmWareVersion = 0,      //0, Version # set at top of main.c
  RegUniqueID,                 //1, a unique ID for each MCU (96 bits, but only returns lowest 16 bits)
  RegTick,                     //2, lower 16 bits of the 1mS system timer
  RegAdcTemp,                  //3, read temperature sensor internal to MCU, 0.76V @25degC + 2.5mV/degC, temp = 25+(nAdc*3.3/4095-0.76)/0.025, 1022 => 27.5degC
  RegAdcRef,                   //4, MCU internal reference voltage (1.2Vnom), should be around 1.2/3.3*4095 = 1500, can use to calculate voltage of 3.3V supply
	RegPeltierTempColdsideAdc,			//5, when read, takes an adc measurement of the thermister on cold side of heat pump
	RegPeltierTempColdsideCelsius,		//6	when read, takes an adc measurement of the thermister on cold side of heat pump and converts that temp into celsius !!currently broken. Will also need to handle negative values
	RegPeltierTempHotsideAdc,			//7, when read, takes an adc measurement of the thermister on hot side of heat pump
	RegPeltierTempHotsideCelsius,		//8	when read, takes an adc measurement of the thermister on hot side of heat pump and converts that temp into celsius !!currently broken. Will also need to handle negative values
	RegPeltierTempBoxAdc,				//9, when read, takes an adc measurement of the thermister inside the insulated box
	RegPeltierTempBoxCelsius,			//10 when read, takes an adc measurement of the thermister on cold side of heat pump an
	RegPeltierPWMDutyCycle,				//11 when written to, changes the PWM duty cycle to whatever percentage was entered
	RegPeltierReverseMode,				//12, if set to 0 (default), run in normal mode; if 1, reverse heat transfer direction. Note that 100% duty cycle is maximum heat transfer in normal but slowest in reverse mode. 
	RegPeltierBangBangTempTargetAdc,	//13, enabled by uncommenting BangBangControl macro in main.h; Target adc value: if current temp adc is below this value PWM turns/stays on when met or exceeded PWM turns off.
	RegPeltierBangBangTempAllowanceAdc,	//14, enabled by uncommenting BangBangControl macro in main.h; The interval below target adc where PWM will remain off even though current temp is below target. PWM will turn on again once current temp adc drops below Target-Allowance
	RegPeltierPidSetpointAdc,			//15, target temp adc for the pid loop to achieve
	RegPeltierPidDt,					//16, units of ms; recalculate error value every dt ms
	RegPeltierPidKp,					//17, constant of proporionality controlling the proportional term
	RegPeltierPidKi,					//18, constant of proporionality controlling the integral term
	RegPeltierPidISum,
	RegPeltierPidISumMax,
	RegPeltierPidKd,					//19, constant of proporionality controlling the derivative term
	RegPeltierPidMv,					//20, 
	RegPeltierForceTemp,				//21, when nonzero, overrides the PID controller to rush 100% to the setpoint. Then stops when it is within a few adc values, returning control to PID controller. Stopping override is handled by ReadReg(RegPeltierTempColdsideAdc)
	RegLast
};

	
/* Exported functions prototypes ---------------------------------------------*/
	void SystemClock_Config(void);
void InitRegs();
uint8_t UpdateRegs();
uint8_t ReadReg(uint16_t nReg);
void SetReg(uint16_t nReg, uint16_t value);
uint16_t ReadAdc(uint16_t chan);

#ifdef __cplusplus
}
#endif

#endif // __REGS_H__