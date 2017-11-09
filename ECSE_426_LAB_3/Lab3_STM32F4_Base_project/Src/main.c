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
#include "math.h"

#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_tim.h>


LIS3DSH_InitTypeDef 		Acc_instance;
LIS3DSH_DRYInterruptConfigTypeDef interruptCfg;
SPI_HandleTypeDef *hspi;
/* Private variables ---------------------------------------------------------*/

uint8_t status;

// THIS IS THE BUFFER FOR THE INPUT OF THE ACCELEROMETER SENSOR.
float inputBuffer[3];

// THESE VARIABLES STORE THE CURRENT RAW VALUES FROM THE ACCELEROMETER SENSOR.
float accX, accY, accZ;

// THESE CONSTANTS ARE USED TO CALIBRATE THE VALUES OF THE ACCELEROMETER SENSOR.
float	acc11 = 0.00101492331;
float	acc12 = 0.00000698307;
float	acc13 = 0.00000805562;
float	acc10 = 0.00221516886;
float	acc21 = -0.00007170752;
float	acc22 = 0.00095980401;
float	acc23 = 0.00002352682;
float	acc20 = 0.01772957335;
float	acc31 = -0.00010254170;
float	acc32 = -0.00001399401;
float	acc33 = 0.00098730978;
float	acc30 = -0.00523204952;

// THESE VARIABLES STORE THE CURRENT CALIBRATED VALUES.
float normalizedAccX, normalizedAccY, normalizedAccZ;

// THESE ARRAYS KEEP TRACK OF THE LAST THREE UNFILTERED ACCELEROMETER READINGS FOR A PARTICULAR AXIS: {x[n], x[n-1], x[n-2]}.
float unfilteredAccX[3] = {0.0, 0.0, 0.0};
float unfilteredAccY[3] = {0.0, 0.0, 0.0};
float unfilteredAccZ[3] = {0.0, 0.0, 0.0};

// THESE ARRAYS KEEP TRACK OF THE LAST TWO FILTERED ACCELEROMETER VALUES FOR A PARTICULAR AXIS: {y[n-1], y[n-2]}.
float filteredAccX[2] = {0.0, 0.0};
float filteredAccY[2] = {0.0, 0.0};
float filteredAccZ[2] = {0.0, 0.0};

// THESE CONSTANTS ARE THE FILTER COEFFICIENTS.
float b0 = 0.2;
float b1 = 0.2;
float b2 = 0.2;
float a1 = 0.2;
float a2 = 0.2;

// THESE VARIABLES STORE THE CURRENT FILTERED AND CALIBRATED VALUES.
float finalAccX, finalAccY, finalAccZ;

// THESE VARIABLES STORE THE CURRENT ROLL AND PITCH ANGLES.
double rollAngle;
double pitchAngle;

// THESE VARIABLES STORE THE TARGET ROLL AND PITCH ANGLES (ENTERED ON THE KEYPAD).
double targetRollAngle  = 45.0;
double targetPitchAngle = 45.0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void initializeACC			(void);
int SysTickCount;

int keyPressedCounter;
int currentKeyPressed;
int timer2Counter;
int currentRow;
int currentColumn;

int main(void)
{
	currentKeyPressed = -1;
	keyPressedCounter = 0;
	timer2Counter = 0;
	
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
	
  }
}

/** System Clock Configuration
	The clock source is configured as external at 168 MHz HCLK
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// READ THE ACCELEROMETER VALUES.
	LIS3DSH_ReadACC(&inputBuffer[0]);
	accX = (float)inputBuffer[0];
	accY = (float)inputBuffer[1];
	accZ = (float)inputBuffer[2];
	
	// CALIBRATE THE ACCELEROMETER VALUES.
	normalizedAccX = (accX * acc11) + (accY * acc12) + (accZ * acc13) + acc10;
	normalizedAccY = (accX * acc21) + (accY * acc22) + (accZ * acc23) + acc20;
	normalizedAccZ = (accX * acc31) + (accY * acc32) + (accZ * acc33) + acc30;
	
	// FILTER THE ACCELEROMETER VALUES.
	unfilteredAccX[2] = unfilteredAccX[1];
	unfilteredAccX[1] = unfilteredAccX[0];
	unfilteredAccX[0] = normalizedAccX;
	unfilteredAccY[2] = unfilteredAccY[1];
	unfilteredAccY[1] = unfilteredAccY[0];
	unfilteredAccY[0] = normalizedAccY;
	unfilteredAccZ[2] = unfilteredAccZ[1];
	unfilteredAccZ[1] = unfilteredAccZ[0];
	unfilteredAccZ[0] = normalizedAccZ;
	
	finalAccX = (b0 * unfilteredAccX[0]) + (b1 * unfilteredAccX[1]) + (b2 * unfilteredAccX[2]) + (a1 * filteredAccX[0]) + (a2 * filteredAccX[1]);
	finalAccY = (b0 * unfilteredAccY[0]) + (b1 * unfilteredAccY[1]) + (b2 * unfilteredAccY[2]) + (a1 * filteredAccY[0]) + (a2 * filteredAccY[1]);
	finalAccZ = (b0 * unfilteredAccZ[0]) + (b1 * unfilteredAccZ[1]) + (b2 * unfilteredAccZ[2]) + (a1 * filteredAccZ[0]) + (a2 * filteredAccZ[1]);
	
	filteredAccX[1] = filteredAccX[0];
	filteredAccX[0] = finalAccX;
	filteredAccY[1] = filteredAccY[0];
	filteredAccY[0] = finalAccY;
	filteredAccZ[1] = filteredAccZ[0];
	filteredAccZ[0] = finalAccZ;
	
	// COMPUTE THE ROLL AND PITCH ANGLES.
	rollAngle  = (180/3.141592654) * atan2f(((-1) * finalAccX), finalAccZ);
	pitchAngle = (180/3.141592654) * atan2f(finalAccY, finalAccZ);
	
	if(rollAngle < 0)
	{
		 rollAngle = rollAngle + 360;
	}
	if(pitchAngle < 0)
	{
		 pitchAngle = pitchAngle + 360;
	}
	
	double deltaRoll = rollAngle - targetRollAngle;
	double deltaPitch = pitchAngle - targetPitchAngle;
	
	if(deltaRoll < 0)
	{
		deltaRoll = deltaRoll * -1;
	}
	if(deltaPitch < 0)
	{
		deltaPitch = deltaPitch * -1;
	}
	
	// IF THE ACTUAL ANGLE IS WHITHIN 5 DEGREES OF THE TARGET ANGLE, YOU WIN!!!
	if(deltaRoll < 5)
	{
		deltaRoll = 0;
	}
	if(deltaPitch < 5)
	{
		deltaPitch = 0;
	}
	
	// UPDATE THE BRIGHTNESS OF THE LEDS. RED AND GREEN ROLL, BLUE AND ORANGE FOR PITCH.
	updatePulse(deltaRoll,  TIM_CHANNEL_3, &htim4);
	updatePulse(deltaRoll,  TIM_CHANNEL_1, &htim4);
	updatePulse(deltaPitch, TIM_CHANNEL_4, &htim4);
	updatePulse(deltaPitch, TIM_CHANNEL_2, &htim4);
	
	//printf(" %3f , %3f \n", rollAngle, pitchAngle);
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	
  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 336;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
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
	// TIMER 2: This interrupt gets called every 10 ms.
  if(htim == &htim2)
  {
		
		if(timer2Counter == 0)
		{
			//currentRow = getPressedRow();
			timer2Counter++;
		}
		else
		{
			//currentColumn = getPressedColumn();
			timer2Counter = 0;
		}
		
		//printf("TIM2 calleback");
		currentRow = getPressedRow();
		currentColumn = getPressedColumn();
		int key = getPressedKey(currentRow, currentColumn);
		printf(" current key pressed is: %d  \n", key);
		
		if(key == currentKeyPressed && key != -1)
		{
			keyPressedCounter++;
		}
		else if(key != currentKeyPressed && key ==1)
		{
			//printf(" current key pressed is: %d \n counter: %d  \n", currentKeyPressed, keyPressedCounter);
		}
	}

}
/* USER CODE END 4 */
void initializeACC(void)
{
	Acc_instance.Axes_Enable				        = LIS3DSH_XYZ_ENABLE;
	Acc_instance.AA_Filter_BW				        = LIS3DSH_AA_BW_50;
	Acc_instance.Full_Scale					        = LIS3DSH_FULLSCALE_2;
	Acc_instance.Power_Mode_Output_DataRate	= LIS3DSH_DATARATE_50;
	Acc_instance.Self_Test					        = LIS3DSH_SELFTEST_NORMAL;
	Acc_instance.Continous_Update           = LIS3DSH_ContinousUpdate_Enabled;
	
	LIS3DSH_Init(&Acc_instance);	
	
	/* Enabling interrupt conflicts with push button. Be careful when you plan to 
	use the interrupt of the accelerometer sensor connceted to PIN A.0 */
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
