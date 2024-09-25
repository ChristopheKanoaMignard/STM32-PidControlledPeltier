/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "hwConfig.h"
#include "regs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "math.h"


/* Private variables ---------------------------------------------------------*/
uint16_t FirmWareVersion = 1;
uint8_t txBuf[2048];			//to receive data over usb_user
uint8_t rxBuf[2048];			//to receive data over usb_user
uint32_t rxLen = 0;				//to receive data over usb_user
uint32_t fifoCounter = 0;
uint32_t innerPidCounter = 0;
uint8_t outerPidFreq = 3;		//The outer pid calculation occurs at 1/{value} * frequency of the inner pid. Inner pid frequency is tempBoxcarPeriod * fifMaxIndex.
uint16_t hotsideFifo[fifoMaxIndex];
uint16_t coldsideFifo[fifoMaxIndex];
uint16_t boxFifo[fifoMaxIndex];
uint16_t tempBoxcarPeriod = 30;	//Every {value} ms the adc polls all thermister values. 
uint32_t sysTick = 0;		//a timer that counts up until tempBoxcarPeriod has been reached. 	
uint32_t lastCalcTick = 0;	//a timer that records when the previous temperature adc conversion occured. 
int32_t integralSummation = 0;
int boolPidOverride = 0;		//Flag that prevents pid controller from making calculations or changing pwm. Set when RegPeltierForceTemp is written to be nonzero and gets reset once coldside adc reaches written value 

/* Private function prototypes -----------------------------------------------*/
static void BlinkGreenLed(void);
void PidControlCalculation(void);
void PidOuterControlCalculation(void);
uint16_t AverageAdc(uint8_t len, uint16_t array[len]);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_NVIC_Init();
	MX_TIM3_Init();
	MX_USB_DEVICE_Init();
	InitRegs();
	SetReg(RegPeltierPidDt, (tempBoxcarPeriod*fifoMaxIndex));		//this dt in ms is likley too big and causes issues with pid calculations. Convert to seconds inside controller
	
	HAL_GPIO_WritePin(DriverEnable_GPIO_Port, DriverEnable_Pin, GPIO_PIN_SET);		//enables L298 board	
	HAL_GPIO_WritePin(DriverInput2_GPIO_Port, DriverInput2_Pin, GPIO_PIN_RESET);	//normal heat pumping direction
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);		//start pwm that drives current through peltier



	/*
	 *Populate all elements of temperature boxcar-averaging arrays with temperatures. 
	 *Then set their regs to the average of the array elements. 
	 **/
	for (fifoCounter = 0; fifoCounter < fifoMaxIndex; fifoCounter++) {
		boxFifo[fifoCounter] = ReadAdc(11);				//box
		hotsideFifo[fifoCounter] = ReadAdc(12);			//hot
		coldsideFifo[fifoCounter] = ReadAdc(13);		//cold
	}	
	SetReg(RegPeltierTempBoxAdc, AverageAdc(fifoMaxIndex, boxFifo));
	SetReg(RegPeltierTempHotsideAdc, AverageAdc(fifoMaxIndex, hotsideFifo));
	SetReg(RegPeltierTempColdsideAdc, AverageAdc(fifoMaxIndex, coldsideFifo));
	
	while (1)
	{

		//BlinkGreenLed();
		
		sysTick = HAL_GetTick();
		
#ifdef PidControl		
		if ((sysTick - lastCalcTick) > tempBoxcarPeriod) //Run another calculation once per tempBoxcarPeriod ms
		{
			if ((boolPidOverride) == 0   &&   ((fifoCounter % fifoMaxIndex) == 0))		//Once the entire array of temperature values has been replaced exactly once: perform new pid calculation
			{
				PidControlCalculation();	//Perform another pid controller calculation
				innerPidCounter++;
				
				if (innerPidCounter % 3 == 0)
				{
					PidOuterControlCalculation();
				}
			}

				
			//Take new temperature data, then boxcar average them. This average is stored in registers and is used for controller calculation and read by gui
			boxFifo[(fifoCounter % fifoMaxIndex)] = ReadAdc(11);			
			SetReg(RegPeltierTempBoxAdc, AverageAdc(fifoMaxIndex, boxFifo));
			hotsideFifo[(fifoCounter % fifoMaxIndex)] = ReadAdc(12);	
			SetReg(RegPeltierTempHotsideAdc, AverageAdc(fifoMaxIndex, hotsideFifo));
			coldsideFifo[(fifoCounter % fifoMaxIndex)] = ReadAdc(13);			
			SetReg(RegPeltierTempColdsideAdc, AverageAdc(fifoMaxIndex, coldsideFifo));
			fifoCounter++;

			lastCalcTick = sysTick;
		}
#endif	
		
		if (rxLen > 0) {
			Parse();
		}
	}
}

void BlinkGreenLed()
{
	//period is blinkTime + blinkOnTime = 889*(1+1/8) = 1000
	static uint32_t blinkTime = 889; //off time
	static uint8_t lastBlink = 0;
	static uint8_t bBlinkOn = 1;
	uint32_t blinkOnTime = blinkTime >> 3; //on time
	uint32_t sysTick = HAL_GetTick();
  
	if ((sysTick - lastBlink) > blinkOnTime) {
		if (bBlinkOn) {
			// LED off
			HAL_GPIO_WritePin(GPIOG, Pin_LedGreen, GPIO_PIN_RESET);
			bBlinkOn = 0;
			lastBlink = sysTick;
		}
		else if ((sysTick - lastBlink) > blinkTime) {
			// LED on
			HAL_GPIO_WritePin(GPIOG, Pin_LedGreen, GPIO_PIN_SET);
			bBlinkOn = 1;
			lastBlink = sysTick;
		}
	}
}

/*----------------------------------------Start new project content here----------------------------------------*/

void PidControlCalculation(void)	
{
	static int32_t errorValue = 0, lastErrorValue = 0;
	static uint16_t pwmCcr = 0;						//Set the pwm capture compare register to this value
	const static uint16_t pwmCcrMaxCool = (uint16_t) (0.75 * 0xFFFF);		//Theoretical max the pwm CCR can hold is 0xFFFF. However, this much current will quickly overheat the driver board heat sink. For now, simply cap the max pwm duty cycle: 50% keeps the heatsink from getting hot to the touch. Should replace this with a temperature sensative interlock. 
	const static uint16_t pwmCcrMaxHeat = 0xFFFF - pwmCcrMaxCool;			//If max cooling is .75*0xFFFF, then max heating should be 0xFFFF - (.75*0xFFFF)
	
	errorValue = Regs.u16[RegPeltierPidSetpointAdc] - Regs.u16[RegPeltierTempColdsideAdc];
	
	//Temporarily store summation in variable rather than reg in case data is read by gui and |integralSummation| > Regs.u16[RegPeltierPidISumMax] but before the maximum gets enforced. Unlikely but theoretically possible
	integralSummation = integralSummation + Regs.u16[RegPeltierPidKi] * errorValue * (0.001*Regs.u16[RegPeltierPidDt]);		//Integral windup is handled by SetReg(RegPeltierPidSetpointAdc). Convert dt from ms to s
	if (integralSummation > Regs.s32[RegPeltierPidISumMax]) {
		integralSummation = Regs.s32[RegPeltierPidISumMax];
	}
	else if (integralSummation < (-1 * Regs.s32[RegPeltierPidISumMax])) {
		integralSummation = -1 * Regs.s32[RegPeltierPidISumMax];
	}
	Regs.s32[RegPeltierPidISum] = integralSummation;
	
	Regs.s32[RegPeltierPidMv] =	(int64_t)(Regs.u16[RegPeltierPidKp] * errorValue + 
											Regs.s32[RegPeltierPidISum] + 
											Regs.u16[RegPeltierPidKd] * (errorValue - lastErrorValue) / Regs.u16[RegPeltierPidDt]);
	lastErrorValue = errorValue;
		
	if (Regs.s32[RegPeltierPidMv] >= 0)		//If MV is positive or 0, then peltier should be in normal mode and maximal cooling is 100% duty
	{	
		SetReg(RegPeltierReverseMode, 0);				//Run peltier in normal mode ie cooling. 
		if (Regs.s32[RegPeltierPidMv] > pwmCcrMaxCool) {		
			pwmCcr = pwmCcrMaxCool;										//Maximum allowed cooling.
		}
		else {
			pwmCcr = (uint16_t) Regs.s32[RegPeltierPidMv];				//Partial cooling
		}


	}
	else													//If MV is negative, then peltier should be in reverse mode and maximal heating is 0% duty cycle
	{
		SetReg(RegPeltierReverseMode, 1);					//Run peltier in reverse mode.
			
		//Regs.s32[RegPeltierPidMv] = Regs.s32[RegPeltierPidMv] + 0xFFFF;	//This is better practice computationally, but it corrupts data taking by altering the Mv stored
		if ((Regs.s32[RegPeltierPidMv]+0xFFFF) < pwmCcrMaxHeat) {				
			pwmCcr = pwmCcrMaxHeat;														//Maximum allowed heating
		}
		else {
			pwmCcr = (uint16_t) (Regs.s32[RegPeltierPidMv] + 0xFFFF);					//Partial heating
		}
	}

	TIM3->CCR1 = pwmCcr;
	//SetReg(RegPeltierPidManipulatedVariable, (uint16_t)(manipulatedVariable));	
	
	
}

void PidOuterControlCalculation(void)	
{
	static int64_t manipulatedVariableOuter = 0;
	static int32_t errorValueOuter, lastErrorValueOuter = 0;
}

uint16_t AverageAdc(uint8_t len, uint16_t array[len])
{
	uint32_t sum = 0;
	for (uint8_t ij = 0; ij < len; ij++){
		sum += array[ij];
	}
	
	return (uint16_t)(sum / len);
}


