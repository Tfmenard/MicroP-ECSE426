/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>

#define DIGIT_1 GPIO_PIN_11
#define DIGIT_2 GPIO_PIN_0
#define DIGIT_3 GPIO_PIN_7
#define DIGIT_4 GPIO_PIN_7

#define SEGMENT_A GPIO_PIN_10
#define SEGMENT_B GPIO_PIN_12
#define SEGMENT_C GPIO_PIN_9
#define SEGMENT_D GPIO_PIN_2
#define SEGMENT_E GPIO_PIN_6
#define SEGMENT_F GPIO_PIN_8
#define SEGMENT_G GPIO_PIN_11
#define POINT GPIO_PIN_5

#define SQUARE_WAVE_FREQ 10 //50Hz, 17 would be 60Hz

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void incrementVals(int k, int N, uint8_t sample, float* vals, float* pastWindow);
void lightUpDigit(int number, int digitNumber, int point);
float getKVrms(int k, int N, float* vals);



int k = 0;
int windowSize = 1000;
int captureRateCounter = 0;
float *vals;
float *pastWindow;

uint16_t adcVal;
	
	//int adcVal = HAL_ADC_GetValue(&hadc2);
	//incrementVals(k, windowSize, adcVal, vals, pastWindow);
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4

/*

// need to configure ADC
// adc_channel -> 0,1,2
// adc_resolution -> 8 bits
// adc_mode -> normal, independent


in interrupt function HAL_

what is the scan conversion mode, what is the (dma)? ... demo shit adc

do this every time you wanna poll the adc
HAL_TIM_PeriodElapsedCallback {
	show (ADC value)
}


adc conversion mode -> external trig source -> regular
this is the polling option


the right way is using timer + interrupt


*/


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start(&hadc2);
	float tempVals[2];
	vals = tempVals;
	float tempPastWindow[2];
	pastWindow = tempPastWindow;
	
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
	
  while (1)
  {
		if (HAL_ADC_PollForConversion(&hadc2, 1000000) == HAL_OK)
        {
					// spike filtering logic applied directly to polled values from the adc
					// use the fact that we are measuring a square wave to remove spikes. the adc values should only be 0 or 255 in theory
					// if a value's variation is not either very small or very big, discard the value
					
						uint16_t pastAdc = adcVal;
						uint16_t newAdcVal = HAL_ADC_GetValue(&hadc2);
						//printf("%d\n", newAdcVal);
						int difference = abs(pastAdc - adcVal);
						if(difference <=10 || difference>=150){
							adcVal = newAdcVal;
						}
        }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

void showRmsValue(float rms, int index){
	char digits[6];
	sprintf(digits, "%.3f", rms);
	// shift the array so that the "." character is not included in the digits to display
	digits[1] = digits[2];
	digits[2] = digits[3];
	digits[3] = digits[4];
	if(index == 1){
		// display a dot after the first digit
		lightUpDigit(digits[index-1]  - '0', index, 1);
	} else{
		lightUpDigit(digits[index-1]  - '0', index, 0);
	}
}




void lightUpDigit(int number, int digitNumber, int point){
	//Always set off ":"
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	
	if(digitNumber == 1){
		HAL_GPIO_WritePin(GPIOC, DIGIT_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DIGIT_4, GPIO_PIN_SET);
	} else if(digitNumber == 2){
		HAL_GPIO_WritePin(GPIOC, DIGIT_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DIGIT_4, GPIO_PIN_SET);
	} else if(digitNumber == 3){
		HAL_GPIO_WritePin(GPIOC, DIGIT_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, DIGIT_4, GPIO_PIN_SET);
	} else if(digitNumber == 4){
		HAL_GPIO_WritePin(GPIOC, DIGIT_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, DIGIT_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DIGIT_4, GPIO_PIN_RESET);
	}
	switch(number){
		case(0) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_RESET);
			break;
		case(1) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_RESET);
			break;
		case(2) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_SET);
			break;
		case(3) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_SET);
			break;
		case(4) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_SET);
			break;
		case(5) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_SET);
			break;
		case(6) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_SET);
			break;
		case(7) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_RESET);
			break;
		case(8) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_SET);
			break;
		case(9) :
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_SET);
			break;
		default:
			printf("gros");
			HAL_GPIO_WritePin(GPIOE, SEGMENT_A, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_B, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_C, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_D, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, SEGMENT_E, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_F, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SEGMENT_G, GPIO_PIN_RESET);
			break;
	}
	if(point){
		HAL_GPIO_WritePin(GPIOB, POINT, GPIO_PIN_SET);
	} else{
		HAL_GPIO_WritePin(GPIOB, POINT, GPIO_PIN_RESET);
	}
}

float savedRms = 0;
int squareWaveCounter = 0;
int digitCounter = 1;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
	// decrease the sampling rate to 2ms for lower window sizes, so that the window represents a wider part of the square wave function
  incrementVals(k, windowSize, adcVal, vals, pastWindow);
  if(captureRateCounter == 100){
    savedRms = getKVrms(k, windowSize, vals);
    captureRateCounter = 0;
    showRmsValue(savedRms, digitCounter);
    digitCounter++;
  } else if(captureRateCounter != 0 && captureRateCounter % 5 == 0) {
    showRmsValue(savedRms, digitCounter);
    digitCounter++;
  }
  k++;
	if(digitCounter == 5){
		digitCounter = 1;
	}
	captureRateCounter++;
	squareWaveCounter++;
}

void incrementVals(int k, int N, uint8_t sample, float* vals, float* pastWindow){
	//ensure not going to overflow at next increment, consider that val_A or val_B could be largest value
	if(vals[0] > FLT_MAX/2 || vals[1] > FLT_MAX/2){
		float subtract = vals[0] > vals[1] ? vals[1]/2 : vals[0]/2;
		vals[0] -= subtract;
		vals[1] -= subtract;
	}
	// adc returns 255 for the maximum 3.0V, so use this to determine a linear coefficient to scale bit values returned by adc
	float analogVRms = 3.0/255 * sample;
  vals[0] += pow(analogVRms, 2);
  int index = k % N;
  if(k>=N){
   vals[1] += pow(pastWindow[index], 2);
  }
  pastWindow[index] = analogVRms;
}

float getKVrms(int k, int N, float* vals){
	if(vals[0] >= vals[1]){
		return sqrt((vals[0] - vals[1])/N);
	}
	return 0;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
