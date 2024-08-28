/**
  ******************************************************************************
  * @file           : nvParams.h
  * @brief          : Code to handle registers that do survive power cycle
  ******************************************************************************
  */
#ifndef __PARAMS_H__
#define __PARAMS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "regs.h"

extern REG_BLOCK SysParams;

enum SYS_PARAMS {
	NvStart, 					 //0, start code (0x55a0)
	NvBoardType,				 //1, board type, 0=FlashController from 10/2020
	NvHwVersion,				 //2, hardware revision, 0=FlashController from 10/2020
	NvLast        
};

int Flash_Params();
int Save_Param(unsigned short nParam);
void Load_Params(uint8_t isForceDefault); //isForceDefault forces the defaults onto the system
void Set_Parameter(uint16_t nParam, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif // __PARAMS_H__