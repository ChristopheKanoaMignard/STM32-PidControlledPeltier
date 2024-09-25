/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "regs.h"
#include "nvParams.h"
#include "commands.h"	
	
/* Defines -------------------------------------------------------------------*/
//#define BangBangControl
#define PidControl
#define PidControlNested
#define fifoMaxIndex					((uint8_t) 9)		//Boxcar averaging reduces noise by a factor of sqrt(fifoMaxIndex)
/* Exported variables -------------------------------------------------------*/
extern uint32_t adcTemps[2];
extern ADC_HandleTypeDef hadc1;
extern uint16_t hotsideFifo[fifoMaxIndex];
extern uint16_t coldsideFifo[fifoMaxIndex];
extern uint16_t boxFifo[fifoMaxIndex];
extern int32_t integralSummation;
extern int boolPidOverride;
	
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint16_t AverageAdc(uint8_t len, uint16_t array[len]);
	
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
