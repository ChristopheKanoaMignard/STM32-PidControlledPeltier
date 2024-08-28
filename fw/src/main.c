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
#ifdef RtosTest
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#endif

/* Private variables ---------------------------------------------------------*/
uint16_t FirmWareVersion = 1;
uint8_t txBuf[2048];			//to receive data over usb_user
uint8_t rxBuf[2048];			//to receive data over usb_user
uint32_t rxLen = 0;				//to receive data over usb_user
uint32_t lastBlink;				//for blinking hearbeat LED
#ifdef RtosTest
UART_HandleTypeDef huart4;

osThreadId defaultTaskHandle;
osThreadId TestThread1Handle;
osThreadId TestThread2Handle;
osMutexId TestMutex1Handle;
#endif
/* Private function prototypes -----------------------------------------------*/
static void BlinkGreenLed(void);
void PidControlCalculation(void);
#ifdef RtosTest
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);
void StartTestThrea1(void const * argument);
void StartTestThread2(void const * argument);
#endif
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
	MX_NVIC_Init();
	//MX_SPI2_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
#ifdef RtosTest
	MX_UART4_Init();
	/* Create the mutex(es) */
  /* definition and creation of TestMutex1 */
	osMutexDef(TestMutex1);
	TestMutex1Handle = osMutexCreate(osMutex(TestMutex1));
	/* Create the thread(s) */
  /* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of TestThread1 */
	osThreadDef(TestThread1, StartTestThrea1, osPriorityIdle, 0, 128);
	TestThread1Handle = osThreadCreate(osThread(TestThread1), NULL);

	/* definition and creation of TestThread2 */
	osThreadDef(TestThread2, StartTestThread2, osPriorityIdle, 0, 128);
	TestThread2Handle = osThreadCreate(osThread(TestThread2), NULL);
	
	osKernelStart();
#endif
	
	
	
	
	MX_USB_DEVICE_Init();
	InitRegs();
    //Load_Params(0); //this causes a HardFault, probably because reading beyond flash

	HAL_GPIO_WritePin(DriverEnable_GPIO_Port, DriverEnable_Pin, GPIO_PIN_SET);		//enables L298 board	
	HAL_GPIO_WritePin(DriverInput2_GPIO_Port, DriverInput2_Pin, GPIO_PIN_RESET);	//normal heat pumping direction
	//HAL_GPIO_WritePin(DriverInput2_GPIO_Port, DriverInput2_Pin, GPIO_PIN_SET);	//reverse heat pumping direction	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
	while (1)
	{
		//BlinkGreenLed();
		
		PidControlCalculation();
		
		if (rxLen > 0) {
			Parse();
		}
	}
}

void BlinkGreenLed()
{
	//period is blinkTime + blinkOnTime = 889*(1+1/8) = 1000
	static uint32_t blinkTime = 889; //off time
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

void PidControlCalculation(void)	//!!Should set up an interrupt to call this function every dt ms is I want finer resolution
{
	static uint32_t sysTick, lastCalc, manipulatedVariable, errorValue, lastErrorValue = 0;
	static long manipulatedVariableLong = 0;
	sysTick = HAL_GetTick();
  
	if ((sysTick - lastCalc) > Regs.u16[RegPeltierPidDt]) 
	{
		errorValue = Regs.u16[RegPeltierPidSetpointAdc] - Regs.u16[RegPeltierTempColdsideAdc];
		manipulatedVariable = (uint32_t)(	Regs.u16[RegPeltierPidKp] * errorValue + 
											Regs.u16[RegPeltierPidKi] * errorValue * Regs.u16[RegPeltierPidDt] + 
											Regs.u16[RegPeltierPidKd] * (errorValue - lastErrorValue) / Regs.u16[RegPeltierPidDt]);
		if (manipulatedVariable > 0xFFFF) {		//!!This is causing issues. When errorValue < 0, or at least tries to be, we want CCR1 = 0 (and ideally even to run in reverse mode), but right now if temp exceeds SP CCR1 gets locked to 100% DC
			manipulatedVariable = 0xFFFF;
			}
		else if (manipulatedVariable < 0){
			static int negValueDetected;
			negValueDetected++;
			manipulatedVariable = 0;
		}
		TIM3->CCR1 = manipulatedVariable;	//!!Would prefer to handle this in SetReg, need larger data type regs. Works for now.
		SetReg(RegPeltierPidManipulatedVariable, (uint16_t)(manipulatedVariable));	
	}
	
	
	
	
}






#ifdef RtosTest

static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}
/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (huart->Instance == UART4)
	{
		/* USER CODE BEGIN UART4_MspInit 0 */

		/* USER CODE END UART4_MspInit 0 */
		  /* Peripheral clock enable */
		__HAL_RCC_UART4_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		/**UART4 GPIO Configuration
		PA1     ------> UART4_RX
		PC10     ------> UART4_TX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* USER CODE BEGIN UART4_MspInit 1 */

		/* USER CODE END UART4_MspInit 1 */
	}

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	if (huart->Instance == UART4)
	{
		/* USER CODE BEGIN UART4_MspDeInit 0 */

		/* USER CODE END UART4_MspDeInit 0 */
		  /* Peripheral clock disable */
		__HAL_RCC_UART4_CLK_DISABLE();

		/**UART4 GPIO Configuration
		PA1     ------> UART4_RX
		PC10     ------> UART4_TX
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);

		/* USER CODE BEGIN UART4_MspDeInit 1 */

		/* USER CODE END UART4_MspDeInit 1 */
	}

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTestThrea1 */
/**
* @brief Function implementing the TestThread1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestThrea1 */
void StartTestThrea1(void const * argument)
{
	/* USER CODE BEGIN StartTestThrea1 */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartTestThrea1 */
}

/* USER CODE BEGIN Header_StartTestThread2 */
/**
* @brief Function implementing the TestThread2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestThread2 */
void StartTestThread2(void const * argument)
{
	/* USER CODE BEGIN StartTestThread2 */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartTestThread2 */
}








void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM3) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}









#endif





#ifdef RtosTest

#endif