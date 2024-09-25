/**
  ******************************************************************************
  * @file           : hwConfig.h
  * @brief          : Code to configure the MCU devices (hardware)
  ******************************************************************************
  */
#ifndef __HWCONFIG_H__
#define __HWCONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_conf.h"
#include <usbd_core.h>
#include <usbd_cdc.h>
#include "usbd_cdc_if.h"
#include <usbd_desc.h>

/* Exported variables -------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;

/* Exported functions prototypes ---------------------------------------------*/
void SystemClock_Config(void);
void InitUsb(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_NVIC_Init(void);
void MX_SPI2_Init(void);
void MX_TIM3_Init(void);	
	
void MX_DMA_Init(void);

/* Defines ---------------------------------------------------------------------*/	
#define Pin_LedGreen						GPIO_PIN_13
#define Pin_Adc1ColdTherm					GPIO_PIN_3
#define GPIO_Port_Adc1ColdTherm				GPIOC
#define Pin_Adc1HotTherm					GPIO_PIN_2
#define GPIO_Port_Adc1HotTherm				GPIOC
#define Pin_Adc1BoxTherm					GPIO_PIN_1
#define GPIO_Port_Adc1BoxTherm				GPIOC
	
#define DriverInputPWM_Pin					GPIO_PIN_6
#define DriverInputPWM_GPIO_Port			GPIOC
#define DriverEnable_Pin					GPIO_PIN_7
#define DriverEnable_GPIO_Port				GPIOC
#define DriverInput2_Pin					GPIO_PIN_8
#define DriverInput2_GPIO_Port				GPIOC
#ifdef __cplusplus
}
#endif

#endif // __HWCONFIG_H__