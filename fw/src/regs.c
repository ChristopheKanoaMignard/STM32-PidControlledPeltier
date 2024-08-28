/**
  ******************************************************************************
  * @file           : regs.c
  * @brief          : Code to handle registers that do not survive power cycle
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "regs.h"
#include "spi.h"
#include "math.h"
#include "hwConfig.h"
#include "math.h"

volatile REG_BLOCK Regs;
extern uint16_t FirmWareVersion;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;


uint16_t ReadAdc(uint16_t chan);
int boolPeltierIsOn = 0;

void InitRegs() {
	//Power-on intitializations
	Regs.u16[RegPeltierPidKp] = 200;
	Regs.u16[RegPeltierPidKi] = 100;
	Regs.u16[RegPeltierPidKd] = 0;
	Regs.u16[RegPeltierPidSetpointAdc] = 2400;
	Regs.u16[RegPeltierPidDt] = 3;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
		Error_Handler();

}	

//UpdateRegs called periodically in main()
uint8_t UpdateRegs() {
    uint8_t err = 0;
    static uint16_t nReg=0;
    
    err = ReadReg(nReg);
    nReg++;
    if (nReg>=RegLast || nReg>=REG_SIZE16) nReg = 0;
    
    return err;
}

uint8_t ReadReg(uint16_t nReg) {
	//This routine called if a read register command received over USB, or periodically to update volatile registers.
	//In most cases registers are always valid, but for some the value needs to be updated.
    uint16_t temp = 0, limSw = 0;
    uint8_t err = 0;
    
    if (nReg>=RegLast || nReg>=REG_SIZE16) return 0; //not an error == just don't read past the end of the register block
    
    switch (nReg) {
        case RegFirmWareVersion:
	        Regs.u16[nReg] = FirmWareVersion;
            break;
		case RegUniqueID:
			Regs.u16[nReg] = *(uint16_t *)(0x1fff7a10UL);
			break;
		case RegTick:
			Regs.u16[nReg] = (uint16_t)(HAL_GetTick() & 0xFFFF);
	        break;
		case RegAdcTemp:
			Regs.u16[nReg] = ReadAdc(16); //temperature sensor is channel 16, 0.76V @25degC + 2.5mV/degC, temp = 25+(nAdc*3.3/4095-0.76)/0.025, 1022 => 27.5degC
			break;
		case RegAdcRef:
			Regs.u16[nReg] = ReadAdc(17); //internal reference voltage (1.2Vnom), should be around 1.2/3.3*4095 = 1500, can use to calculate voltage of 3.3V supply
			break;
    case RegPeltierTempColdsideAdc:
	    Regs.u16[nReg] = ReadAdc(13); //tempearture sensor attached to heat sink fans using 3V and a 10Kohm voltage divider
#ifdef BangBangControl
	    
	    //If desired temp (adc) has been reached, then turn off cooler
	    if (boolPeltierIsOn == 1 & Regs.u16[RegPeltierTempColdsideAdc] >= Regs.u16[RegPeltierBangBangTempTargetAdc]) {
		    TIM3->CCR1 = (uint32_t)(0);  
		    boolPeltierIsOn = 0;
	    }
	    //If temp (adc) has dropped below the desired allowance threshold (Target-Allowance), turn cooler back on until desired temp is reached again
	    else if(boolPeltierIsOn == 0 & Regs.u16[RegPeltierTempColdsideAdc] < (Regs.u16[RegPeltierBangBangTempTargetAdc] - Regs.u16[RegPeltierBangBangTempAllowanceAdc])) {	
		    TIM3->CCR1 = (uint32_t)(0xFFFF);
		    boolPeltierIsOn = 1;
	    }
#endif
	    break;
    case RegPeltierTempColdsideCelsius:		//!! Should I use ReadAdc(13 or Regs.u16[RegPeltierTempHeatsinkAdc]?	!!//
	    //Regs.u16[nReg] = (uint16_t)(1 / (1 / 3650 * -log(4095.00000001 / ReadAdc(13) - 1) + 1 / 298)) - 273;	//see black lab notebook, pg 52-53 for derrivation using Steinhart–Hart equation
	    Regs.u16[nReg] = (uint16_t)(1 / (1 / 3650 * -log(4100 / ReadAdc(13) - 1) + 1 / 298)) - 273;	//see black lab notebook, pg 52-53 for derrivation using Steinhart–Hart equation

	    break;
    case RegPeltierTempHotsideAdc:
	    Regs.u16[nReg] = ReadAdc(12); //tempearture sensor attached to heat sink fans using 3V and a 10Kohm voltage divider
	    break;
    case RegPeltierTempHotsideCelsius:		//!! Should I use ReadAdc(13 or Regs.u16[RegPeltierTempHeatsinkAdc]?	!!//
	    Regs.u16[nReg] = (uint16_t)(1 / (1 / 3650 * -log(4095.00000001 / ReadAdc(12) - 1) + 1 / 298)) - 273;	//see black lab notebook, pg 52-53 for derrivation using Steinhart–Hart equation
	    break;
    case RegPeltierPWMDutyCycle:		//!! Should I use ReadAdc(13 or Regs.u16[RegPeltierTempHeatsinkAdc]?	!!//
	    Regs.u16[nReg] = (uint16_t)(100 * (TIM3->CCR1) / 0xFFFF);
	    break;
		default:
			break;
    }  
    return err;
}
void SetReg(uint16_t nReg, uint16_t value) 
{   
	//This routine called if a write register command received over USB
	if (nReg >= RegLast || nReg >= REG_SIZE16) return;
    
	switch (nReg) {
	case RegPeltierPWMDutyCycle:
		Regs.u16[nReg] = value;
		TIM3->CCR1 = (uint32_t)(0xFFFF * ((double)(value) / 100));	//value is the desired duty cycle in percentage.
		if (value != 0) {
			boolPeltierIsOn = 1;
		}
		else if (value == 0) {
			boolPeltierIsOn = 0;
		}
		break;
		#ifdef BangBangControl
	case RegPeltierBangBangTempTargetAdc:	
		Regs.u16[nReg] = value;
		if (ReadReg(RegPeltierTempColdsideAdc) < value) {
			TIM3->CCR1 = (uint32_t)(0xFFFF);
			boolPeltierIsOn = 1;
		}
		break;
	case RegPeltierBangBangTempAllowanceAdc:	
		Regs.u16[nReg] = value;
		break;
		#endif
	case RegPeltierReverseMode:		//!! Change the way this works. Throw up a flag when negative Manipulated Variables are detected to enter reverse mode.	!!//
		if (value == 1) {
			HAL_GPIO_WritePin(DriverInput2_GPIO_Port, DriverInput2_Pin, GPIO_PIN_SET);		//reverse heat pumping direction. 100% duty cycle is now off, 0% is maximum heat pumping.
		}
		else if (value == 0) {
			HAL_GPIO_WritePin(DriverInput2_GPIO_Port, DriverInput2_Pin, GPIO_PIN_RESET);	//normal heat pumping direction
		}
		else {
			HAL_GPIO_WritePin(DriverInput2_GPIO_Port, DriverInput2_Pin, GPIO_PIN_RESET);	//normal heat pumping direction
		}
		break;
		#ifdef PidControl
	case RegPeltierPidSetpointAdc:
		Regs.u16[nReg] = value;
		break;
	case RegPeltierPidKp:
		Regs.u16[nReg] = value;
		break;
	case RegPeltierPidDt:
		Regs.u16[nReg] = value;
		break;
	case RegPeltierPidKi:
		Regs.u16[nReg] = value;
		break;
	case RegPeltierPidKd:
		Regs.u16[nReg] = value;
		break;
	case RegPeltierPidManipulatedVariable:		//!! Currently setting TIM3->CCR1 = MV in while(1) of main.c, but would prefer to do it here. 
		Regs.u16[nReg] = value;
		/*
		if (value > 0xFFFF) {
			//this check is probably unnecessary since CCR1 is a 32bit reg even though CCR1 can only usefully go up to 16 bit values. The odds of this PID ever returning a number bigger than 2^16-1 let alone 2^32-1 is very small and might not matter anyways.
			value = 0xFFFF;
		}
		else if (value < 0) {
			//!!this check is only a temporary measure. I need to handle entering reverse mode automatically for pid control to work well
			value = 0;
			asm("bkpt 255");	
		}
		TIM3->CCR1 = value;
		break;
	*/
		#endif
	default :
		break;
	}
	
	ReadReg(nReg);
}



uint16_t ReadAdc(uint16_t chan)
{
	ADC_ChannelConfTypeDef sConfig = { 0 };
	if (13 == chan)
		sConfig.Channel = ADC_CHANNEL_13;
	else if (12 == chan)
		sConfig.Channel = ADC_CHANNEL_12;
	//else if (16 == chan)
		//sConfig.Channel = ADC_CHANNEL_16;
	//else if (17 == chan)
		//sConfig.Channel = ADC_CHANNEL_17;
	else
		sConfig.Channel = chan;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	uint16_t retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (retVal);
}


