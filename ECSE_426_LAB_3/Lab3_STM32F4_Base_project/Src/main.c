/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "lis3dsh.h"
#include "tim.h"
#include "keypad4x4.h"

#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_tim.h>


LIS3DSH_InitTypeDef 		Acc_instance;
LIS3DSH_DRYInterruptConfigTypeDef interruptCfg;
SPI_HandleTypeDef *hspi;
/* Private variables ---------------------------------------------------------*/

uint8_t status;
float Buffer[3];
float accX, accY, accZ;
float normalizedAccX, normalizedAccY, normalizedAccZ;

float acc11, acc12, acc13, acc10,
			acc21, acc22, acc23, acc20,
			acc31, acc32, acc33, acc30;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void initializeACC			(void);
int SysTickCount;

int keyPressedCounter;
int currentKeyPressed;

int main(void)
{
	acc11 = 0.00101492331;
	acc12 = 0.00000698307;
	acc13 = 0.00000805562;
	acc10 = 0.00221516886;
	
	acc21 = -0.00007170752;
	acc22 = 0.00095980401;
	acc23 = 0.00002352682;
	acc20 = 0.01772957335;
	
	acc31 = -0.00010254170;
	acc32 = -0.00001399401;
	acc33 = 0.00098730978;
	acc30 = -0.00523204952;
	
	currentKeyPressed = -1;
	keyPressedCounter = 0;
	
	
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
	
	MX_TIM4_Init_Alt();
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim4); 
	
	// Green LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	// Orange LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	// Red LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	// Blue LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


	
  while(1)
  {

	/* this is just an example of reading the Accelerometer data in polling technique. You are
		required to read value in interrupt mode automatically, without requesting for a new data every time.
		In fact, the Accl IC will generate data at a certain rate that you have to configure it.
	*/
	// an example of pulse division.
//		if(SysTickCount ==20) 
//		{			
//				LIS3DSH_Read (&status, LIS3DSH_STATUS, 1);
//				//The first four bits denote if we have new data on all XYZ axes, 
//		   	//Z axis only, Y axis only or Z axis only. If any or all changed, proceed
//				if((status & 0x0F) != 0x00)
//				{
//			
//					LIS3DSH_ReadACC(&Buffer[0]);
//					accX = (float)Buffer[0];
//					accY = (float)Buffer[1];
//					accZ = (float)Buffer[2];
//					printf("X: %3f   Y: %3f   Z: %3f  absX: %d\n", accX, accY, accZ , (int)(Buffer[0]));
//					HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
//				}
//			SysTickCount = 0;
//			}
		
  }

}

/** System Clock Configuration
	The clock source is configured as external at 168 MHz HCLK
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	
	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_SET);
	
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14, GPIO_PIN_SET);
	
	
	
	LIS3DSH_ReadACC(&Buffer[0]);
	accX = (float)Buffer[0];
	accY = (float)Buffer[1];
	accZ = (float)Buffer[2];
	//printf(" %3f   , %3f   , %3f  \n", accX, accY, accZ );
	
	normalizedAccX = accX*acc11 + accY*acc12 + accZ*acc13 + acc10;
	normalizedAccY = accX*acc21 + accY*acc22 + accZ*acc23 + acc20;
	normalizedAccZ = accX*acc31 + accY*acc32 + accZ*acc33 + acc30;
	
	//printf(" %3f   , %3f   , %3f  \n", normalizedAccX, normalizedAccY, normalizedAccZ );
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	
  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIMER 2: This gets called every 1 ms.
  if(htim == &htim2)
  {
		//printf("TIM2 calleback");
		int key = getPressedKey();
		
		if(key == currentKeyPressed && key != -1)
		{
			keyPressedCounter++;
		}
		else if(key != currentKeyPressed && key ==1)
		{
			printf(" current key pressed is: %d \n counter: %d  \n", currentKeyPressed, keyPressedCounter);
		}
	}

}
/* USER CODE END 4 */
void initializeACC(void){
	
	Acc_instance.Axes_Enable				= LIS3DSH_XYZ_ENABLE;
	Acc_instance.AA_Filter_BW				= LIS3DSH_AA_BW_50;
	Acc_instance.Full_Scale					= LIS3DSH_FULLSCALE_2;
	Acc_instance.Power_Mode_Output_DataRate		= LIS3DSH_DATARATE_50;
	Acc_instance.Self_Test					= LIS3DSH_SELFTEST_NORMAL;
	Acc_instance.Continous_Update   = LIS3DSH_ContinousUpdate_Enabled;
	
	LIS3DSH_Init(&Acc_instance);	
	
	/* Enabling interrupt conflicts with push button. Be careful when you plan to 
	use the interrupt of the accelerometer sensor connceted to PIN A.0

	*/
}

void IIR_C(float* inputArray, float* outputArray, int i, float* coeff)
{
	if(i == 0)
	{
		// For the first element in the array: y[0] = b0 * x[0].
		outputArray[i] = coeff[0] * inputArray[i];
	}
	else if(i == 1)
	{
		// For the second element in the array: y[1] = b0 * x[1] + b1 * x[0] + a1 * y[0].
		outputArray[i] = coeff[0] * inputArray[i] + coeff[1] * inputArray[i-1] + coeff[3] * outputArray[i-1];
	}
	else
	{
		// For all other elements in the array: y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2]. + a1 * y[n-1] + a2 * y[n-2].
		outputArray[0] = coeff[0] * inputArray[i] + coeff[1] * inputArray[i-1] + coeff[2] * inputArray[i-2] + coeff[3] * outputArray[i-1] + coeff[4] * outputArray[i-2];
		outputArray[2] = outputArray[1];
		outputArray[1] = outputArray[0];
	}
	return;
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
