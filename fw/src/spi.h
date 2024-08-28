/**
  ******************************************************************************
  * @file           : spi.h
  * @brief          : Code to handle SPI bus, which is used to read external ADC.
  *                   The external MUX to set the current for resistance masurement
  *                   is also controlled here
  ******************************************************************************
  */
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

//This only works because all the CSN pins are on GPIO PORT C
//AD7298BCPZ - RL7, ADC to read resets and LEDs
#define CSN0 GPIO_PIN_0 
//AD7298BCPZ - RL7, ADC to read power supply voltages & motor current
#define CSN1 GPIO_PIN_1  
//AD5231BRUZ100, digital pot to set dummy thermistor
#define CSN2 GPIO_PIN_4 
//MAX31865AAP + T, Platinum RTD ADC to measure temperature
#define CSN3 GPIO_PIN_5 
	
extern uint16_t ReadAdc(uint16_t chan);
extern void SetCSN(uint16_t csPin, uint8_t level);
extern void InitRtdAdc(void);
extern void WriteSpi8(uint16_t csPin, uint8_t reg, uint8_t val);
extern void WriteSpi16(uint16_t csPin, uint8_t reg, uint16_t val);
extern uint8_t ReadSpi8(uint16_t csPin, uint8_t reg);
extern uint16_t ReadSpi16(uint16_t csPin, uint8_t reg);
extern int16_t ReadRtdTemp(void);
extern void SetTempPot(uint16_t val);
extern uint16_t ReadTempPot(void);
extern uint16_t ReadAdc7298(uint16_t adcCSN, uint16_t chanCtrl);

#ifdef __cplusplus
}
#endif

#endif // __SPI_H__