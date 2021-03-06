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
#include "stm32f4xx_hal_uart.h"
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_tim.h>


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

#define SAMPLE_FREQ 8000
#define HEADER_SIZE 512
#define BUFFER_SIZE SAMPLE_FREQ*2*2

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
void writeWavHeader();

int startTransmission = 0;
UART_HandleTypeDef huart2;
uint8_t soundBuffer[HEADER_SIZE + BUFFER_SIZE];

int k = 0;
int windowSize = 1000;
int captureRateCounter = 0;
float *vals;
float *pastWindow;

uint8_t adcVal;
	
	//int adcVal = HAL_ADC_GetValue(&hadc2);
	//incrementVals(k, windowSize, adcVal, vals, pastWindow);
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int transmitted = 0;

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
  
  	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart2);
  
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
	
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  
  while (1)
  {
		if (HAL_ADC_PollForConversion(&hadc2, 1000000) == HAL_OK)
        {
						adcVal = HAL_ADC_GetValue(&hadc2);
          //printf("%d\n", adcVal);
        }
        
    if (startTransmission == 1 && transmitted == 0) {
      if( HAL_UART_Transmit_IT(&huart2, soundBuffer, HEADER_SIZE + BUFFER_SIZE) != HAL_OK)
      {
        printf("NOT OK!");
        HAL_Delay(500);
      }
      else
      {
        printf("Transmit successful \n");
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_Delay(500);    
        //soundBuffer[0]++;
       // soundBuffer[0] = (soundBuffer[0]%100);
        printf("TxValue: %d", soundBuffer[0]);
        printf("number of elements: %d\n", sizeof(soundBuffer)/sizeof(soundBuffer[0]));
        transmitted = 1;
      }
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  startTransmission = 0;
  transmitted = 0;
  HAL_ADC_Stop(&hadc2);
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
int recording = 0;
int digitCounter = 1;
int recordingCounter = HEADER_SIZE;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
  
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1){
    for(int i=0; i < HEADER_SIZE + BUFFER_SIZE; i++){
      soundBuffer[i] = 0;
    }
    HAL_ADC_Start(&hadc2);
    recording = 1;
    recordingCounter = HEADER_SIZE;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  } else if(recording == 1){
    soundBuffer[recordingCounter] = adcVal;
    recordingCounter++;
  }
  if(recordingCounter == HEADER_SIZE + BUFFER_SIZE){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    recording = 0;
    recordingCounter = HEADER_SIZE;
    writeWavHeader();
    startTransmission = 1;
  }
}

void writeWavHeader() {
	// chunkID
	soundBuffer[0] = 'R';
	soundBuffer[1] = 'I';
	soundBuffer[2] = 'F';
	soundBuffer[3] = 'F';

	// file length
	// 2 seconds * 8000 = 16 000
	// 16 000 + 36
  //16000 is 0x80 0x3E
  //+ 36 is 0xA4 0x3E
	// format:	00 00 00 00
	// (16^1)(16^0) (16^3)(16^2) (16^5)(16^4) (16^7)(16^6) 
	soundBuffer[4] = 0x24;
	soundBuffer[5] = 0x7D;
	soundBuffer[6] = 0x00;
	soundBuffer[7] = 0x00;

	// file format
	soundBuffer[8]  = 'W';
	soundBuffer[9]  = 'A';
	soundBuffer[10] = 'V';
	soundBuffer[11] = 'E';

	// format chunk
	soundBuffer[12] = 'f';
	soundBuffer[13] = 'm';
	soundBuffer[14] = 't';
	soundBuffer[15] = ' ';

	// length of 'fmt' data
	soundBuffer[16] = 0x10;
	soundBuffer[17] = 0x00;
	soundBuffer[18] = 0x00;
	soundBuffer[19] = 0x00;

	// audio format (PCM)
	soundBuffer[20] = 0x01;
	soundBuffer[21] = 0x00;

	// number of channels (0x01 = mono)
	soundBuffer[22] = 0x02;
	soundBuffer[23] = 0x00;

	// sample rate in Hz
	soundBuffer[24] = (uint8_t)((SAMPLE_FREQ & 0xFF));
  soundBuffer[25] = (uint8_t)((SAMPLE_FREQ >> 8) & 0xFF);
  soundBuffer[26] = (uint8_t)((SAMPLE_FREQ >> 16) & 0xFF);
	soundBuffer[27] = (uint8_t)((SAMPLE_FREQ >> 24) & 0xFF);

	// byte rate
	soundBuffer[28] = (uint8_t)((SAMPLE_FREQ & 0xFF));
  soundBuffer[29] = (uint8_t)((SAMPLE_FREQ >> 8) & 0xFF);
  soundBuffer[30] = (uint8_t)((SAMPLE_FREQ >> 16) & 0xFF);
	soundBuffer[31] = (uint8_t)((SAMPLE_FREQ >> 24) & 0xFF);

	// block alignment
	soundBuffer[32] = 0x02;
	soundBuffer[33] = 0x00;

	// bits per sample
	soundBuffer[34] = 0x08; //0x08 for 8 bits
	soundBuffer[35] = 0x00;

	// data chunk
	soundBuffer[36] = 'd';
	soundBuffer[37] = 'a';
	soundBuffer[38] = 't';
	soundBuffer[39] = 'a';

	// number of sample data (file length - 36)
  //16000 is 0x80 0x3E
	soundBuffer[40] = 0x00;
	soundBuffer[41] = 0x7D;
	soundBuffer[42] = 0x00;
	soundBuffer[43] = 0x00;

	// fill all the remaining bytes with 0x80
	for (int i = 44; i < HEADER_SIZE; i++)
		soundBuffer[i] = 0x80;

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
