/**
  ******************************************************************************
  * @file           : spi.c
  * @brief          : Code to handle SPI bus, which is used to access the following devices:
  *						CSN0, PC0, AD7298BCPZ-RL7, ADC to read resets and LEDs
  *						CSN1, PC1, AD7298BCPZ-RL7, ADC to read power supply voltages & motor current 
  *						CSN2, PC4, AD5231BRUZ100, digital pot to set dummy thermistor
  *						CSN3, PC5, MAX31865AAP+T, Platinum RTD ADC to measure temperature
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "regs.h"
#include "spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "math.h"

extern SPI_HandleTypeDef hspi2;

uint16_t ReadAdc(uint16_t chan);
void SetChannel(uint16_t chan);
void SetCSN(uint16_t csPin, uint8_t level);
void WriteSpi8(uint16_t csPin, uint8_t reg, uint8_t val);
void WriteSpi16(uint16_t csPin, uint8_t reg, uint16_t val);
uint8_t ReadSpi8(uint16_t csPin, uint8_t reg);
uint16_t ReadSpi16(uint16_t csPin, uint8_t reg);
void InitRtdAdc(void);
int16_t ReadRtdTemp(void);
void SetTempPot(uint16_t val);
uint16_t ReadtTempPot(void);
uint16_t ReadAdc7298(uint16_t adcCSN, uint16_t chanCtrl);

void SetCSN(uint16_t csPin, uint8_t level)
{
	//CSN0, PC0, AD7298BCPZ-RL7, ADC to read resets and LEDs
	//CSN1, PC1, AD7298BCPZ-RL7, ADC to read power supply voltages & motor current
	//CSN2, PC4, AD5231BRUZ100, digital pot to set dummy thermistor
	//CSN3, PC5, MAX31865AAP+T, Platinum RTD ADC to measure temperature
	
	if (level)
	{
		HAL_GPIO_WritePin(GPIOC, csPin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, csPin, GPIO_PIN_RESET);
	}
}

//write a byte to one reg in the RTD ADC
void WriteSpi8(uint16_t csPin, uint8_t reg, uint8_t val)
{
	uint8_t controlwords[2];
	controlwords[0] = 0x80 | reg;
	controlwords[1] = val;    //value to write to register
	SetCSN(csPin, 0);
	HAL_SPI_Transmit(&hspi2, controlwords, 2, 1000);
	SetCSN(csPin, 1);
}

//write a word on SPI
void WriteSpi16(uint16_t csPin, uint8_t reg, uint16_t val)
{
	uint8_t controlwords[3];
	controlwords[0] = 0x80 | reg;
	controlwords[1] = (val >> 8) & 0xff;    //value to write to register
	controlwords[2] = val & 0xff;    //value to write to register
	SetCSN(csPin, 0);
	HAL_SPI_Transmit(&hspi2, controlwords, 3, 1000);
	SetCSN(csPin, 1);
}

//read a byte on SPI
uint8_t ReadSpi8(uint16_t csPin, uint8_t reg)
{
	uint8_t controlwords[2];
	controlwords[0] = reg; //must be less than 0x7f or it will be interpreted as a write
	uint8_t rcvDat[2];
	SetCSN(csPin, 0);
	HAL_SPI_TransmitReceive(&hspi2, controlwords, rcvDat, 2, 1000);
	SetCSN(csPin, 1);
	return (rcvDat[1]);
}

//read unsigned short on SPI
uint16_t ReadSpi16(uint16_t csPin, uint8_t reg)
{
	uint8_t controlwords[3];
	controlwords[0] = reg; //must be less than 0x7f or it will be interpreted as a write
	uint8_t rcvDat[3];
	SetCSN(csPin, 0);
	HAL_SPI_TransmitReceive(&hspi2, controlwords, rcvDat, 3, 1000);
	SetCSN(csPin, 1);
	return ((rcvDat[1] << 8) | rcvDat[2]);
}

//set the control register in the RTD sensor
void InitRtdAdc(void)
{
	//read some dummy words because the STM32F starts out with wrong polarity on clock
	uint8_t temp = ReadSpi8(CSN3, 0); 
	temp = ReadSpi8(CSN3, 0);

	//TODO: this board uses 2-wire, ctrl register should be right, but good to verify
	WriteSpi8(CSN3, 0, 0xc2);   // control register, 1100 0010 = vBias on, auto conversion, 2 wire, 60Hz filter
}

//CPOL=1, CPHA=1
int16_t ReadRtdTemp(void)
{
	return ((int16_t)ReadSpi16(CSN3, 1));
}

//set the value of the dummy temperature potentiometer, CPOL=1, CPHA=1, or CPOL=0, CPHA=0
void SetTempPot(uint16_t val)
{
	WriteSpi16(CSN2, 0xb0, val);
}

//The datasheet (https://www.analog.com/media/en/technical-documentation/data-sheets/AD5231.pdf) says
//need write 0x80 00 00 on one frame, then the pot setting will be read back on the next frame, but
//it is not clear if CSn needs to be held continuously low or not.
//Tried both ways, and they both just return 0.
uint16_t ReadTempPot(void)
{
//	WriteSpi16(CSN2, 0xa0, 0);
//	return (ReadSpi16(CSN2, 0xa0));
	
	uint8_t controlwords[6] = { 0, 0, 0, 0, 0, 0 };
	controlwords[0] = 0xa0;
	uint8_t rcvDat[6];
	SetCSN(CSN2, 0);
	HAL_SPI_TransmitReceive(&hspi2, controlwords, rcvDat, 6, 1000);
	SetCSN(CSN2, 1);
	return ((rcvDat[4] << 8) | rcvDat[5]);

}

//Read one of the ADCs. An AD7298 device can be configured to sequentially read a programmed set of channels.
//Here, only one channel is ever selected, and the value is read immediately.
uint16_t ReadAdc7298(uint16_t adcCSN, uint16_t chan)
{
	uint8_t controlwords[2];
	uint8_t rcvDat[2];
	
	//this SPI device requires CPOL=1, CPHA=0, but all the other devices on the bus are CPOL=1, CPHA=1
	//this device is the odd one, so temporarily change CPHA, and then change it back
	//in hardware need to test if the dummy reads are required after changing CPHA
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
		Error_Handler();
	uint8_t temp = ReadSpi8(CSN3, 0); //dummy read required after changing SPI
	
	uint16_t chanCtrl = (0x2000 >> chan); //Encode the channel number into the control word. Channel 8 is internal temperature sensor.
	controlwords[0] = 0x80 | (chanCtrl >> 8);
	controlwords[1] = chanCtrl & 0xff;    //value to write to register
	SetCSN(adcCSN, 0);
	HAL_SPI_Transmit(&hspi2, controlwords, 2, 1000);
	SetCSN(adcCSN, 1);

	SetCSN(adcCSN, 0);
	controlwords[0] &= ~0x80; //already wrote control word, do not need to do it again
	HAL_SPI_TransmitReceive(&hspi2, controlwords, rcvDat, 2, 1000);
	SetCSN(adcCSN, 1);

	SetCSN(adcCSN, 0);
	HAL_SPI_TransmitReceive(&hspi2, controlwords, rcvDat, 2, 1000);
	SetCSN(adcCSN, 1);

	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
		Error_Handler();
	temp = ReadSpi8(CSN3, 0); //dummy read required after changing SPI

	return (((0xf & rcvDat[0]) << 8) | rcvDat[1]); //strip off the upper 4 bits because they encode channel #
}

