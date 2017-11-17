/*******************************************************************************
  * @file    main.c
  * @author  Amirhossein Shahshahani
	* @version V1.2.0
  * @date    10-Nov-2017
  * @brief   This file demonstrates flasing one LED at an interval of one second
	*          RTX based using CMSIS-RTOS 
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "RTE_Components.h"             // Component selection

#include "gpio.h"
#include "lis3dsh.h"
#include "tim.h"
#include "keypad4x4.h"
#include "7_segment.h"
#include "math.h"
#include "main.h"
#include "global_variables.h"

#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_tim.h>

//The thread code is written in Thread_LED.c, just telling the toolchain that the 
//functions are declared externally
extern void initializeLED_IO (void);
extern void start_Thread_LED (void);
extern void Thread_LED (void const *argument);
extern osThreadId tid_Thread_LED;
void SystemClock_Config (void);

/**
	These lines are mandatory to make CMSIS-RTOS RTX work with te new Cube HAL
*/
#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void)
{ 
  return os_time; 
}
#endif

LIS3DSH_InitTypeDef 		Acc_instance;
LIS3DSH_DRYInterruptConfigTypeDef interruptCfg;
SPI_HandleTypeDef *hspi;


uint8_t status;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void initializeACC			(void);
int SysTickCount;

/**
  * Main function
  */
int main (void)
{
  osKernelInitialize();                     /* initialize CMSIS-RTOS          */
  HAL_Init();                               /* Initialize the HAL Library     */
  SystemClock_Config();                     /* Configure the System Clock     */
	/* User codes goes here*/
  //initializeLED_IO();                       /* Initialize LED GPIO Buttons    */
  start_Thread_LED();                       /* Create LED thread              */
	/* User codes ends here*/
	osKernelStart();                          /* start thread execution         */
	
	
	//LAB3CODE STARTS HERE
	interruptCfg.Dataready_Interrupt = LIS3DSH_DATA_READY_INTERRUPT_ENABLED;
	interruptCfg.Interrupt_signal = LIS3DSH_ACTIVE_HIGH_INTERRUPT_SIGNAL;
	interruptCfg.Interrupt_type = LIS3DSH_INTERRUPT_REQUEST_PULSED;
	
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);

	initializeACC	();
	HAL_SPI_MspInit(hspi);
	LIS3DSH_DataReadyInterruptConfig(&interruptCfg);
	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init_Alt();
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim4); 
	
	// Green LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	// Orange LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	// Red LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	// Blue LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	printf("ENTER ROLL ANGLE! \n\n");
  while(1)
  {
	
  }
}

/**
  * System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/* USER CODE END 4 */
void initializeACC(void)
{
	Acc_instance.Axes_Enable				        = LIS3DSH_XYZ_ENABLE;
	Acc_instance.AA_Filter_BW				        = LIS3DSH_AA_BW_50;
	Acc_instance.Full_Scale					        = LIS3DSH_FULLSCALE_2;
	Acc_instance.Power_Mode_Output_DataRate	= LIS3DSH_DATARATE_100;
	Acc_instance.Self_Test					        = LIS3DSH_SELFTEST_NORMAL;
	Acc_instance.Continous_Update           = LIS3DSH_ContinousUpdate_Enabled;
	
	LIS3DSH_Init(&Acc_instance);	
	
	/* Enabling interrupt conflicts with push button. Be careful when you plan to 
	use the interrupt of the accelerometer sensor connceted to PIN A.0 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// THE INTERRUPT OF TIMER 2 GETS CALLED EVERY 7 MS.
  if(htim == &htim2)
  {
		// REFRESH THE SEGMENT DISPLAY AND UPDATE THE DIGIT COUNTER.
		if(FSM_state == 4)
		{
			displayNumberOnTrgtDigit(digitArray[digitCounter], 0);
		}
		else
		{
			displayNumberOnTrgtDigit(digitArray[digitCounter], (digitCounter + 1));
			digitCounter = (digitCounter + 1) % 3;
		}
	}
	else if(htim == &htim3)
	{
		flagForKeypad = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flagForAccelerometer = 1;
}
