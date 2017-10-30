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

/* USER CODE BEGIN Includes */
    
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
  
	// CONSTANT VALUE FOR THE REFERENCE VOLTAGE OF THE ADC.
	double vREF = 3.0;
	
	// THIS VARIABLE KEEPS TRACK OF THE CURRENT ADC SAMPLE.
	int currentADCValue;
	
	// THIS VARIABLE KEEPS TRACK OF THE CURRENT MEASURED VOLTAGE (AFTER DAC CONVERSION).
	double currentAnalogVoltage;
	
	// CONSTANT VALUE FOR THE SIZE OF THE WINDOW.
	int windowSize = 1000;
	
	// BUFFER FOR THE LAST N SAMPLES, WHERE N IS THE WINDOW SIZE.
	double buffer[1000];
	
	// FLAG THAT KEEP TRACK OF WHETHER OR NOT THE BUFFER IS FULL (0 = NO, 1 = YES).
	int bufferFull = 0;

	// THIS VARIABLE KEEPS TRACK OF THE ACCUMULATOR VALUE FOR THE PAST N SAMPLES, WHERE N IS THE WINDOW SIZE.
	double accumulator;
	
	// THIS VARIABLE KEEPS TRACK OF THE SAMPLE NUMBER. IT GETS RESET AFTER EACH N SAMPLES, WHERE N IS THE WINDOW SIZE.
  int sampleCounter = 0;
	
	// THIS VARIABLE KEEPS TRACK OF WHEN TO TOGGLE THE SQUARE WAVE. IT GETS RESET EVERY 10 MS.
  int squareWaveCounter = 0;
	
	// THIS VARIABLE KEEPS TRACK OF WHICH DIGIT OF THE LED DISPLAY SHOULD BE LIT UP.
	int digitCounter = 0;
	
	// THIS ARRAY KEEPS TRACK OF THE VALUE OF THE THREE DIGITS TO DISPLAY (IN THE FORMAT X.XX).
  int digitArray[3] = {1,2,3};
	
	// THIS VARIABLE KEEPS TRACK OF THE CURRENT MEASURED VALUE OF THE RMS VOLTAGE.
	double vRMS;
  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

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
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	
	// Step (1): Start the Timer as interrupt.
	HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
	
	// Step (2): Start the ADC.
	HAL_ADC_Start(&hadc2);
  
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
}	


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIMER 2: This gets called every 1 ms.
  if(htim == &htim2)
  {
		// Get the previous sample of the ADC (since ADC conversion is in process).
		currentADCValue = HAL_ADC_GetValue(&hadc2);
		
		// Check for unexpected spikes and prevent them if they occur.
		currentADCValue = preventUnexpectedSpike(currentADCValue);
		
		// Convert the ADC value to an analog value.
		double voltageValue = (currentADCValue * vREF) / 255;
		
		// If the buffer has not been filled...
		if((sampleCounter < windowSize) && (bufferFull == 0))
		{
			// 1. Update the accumalor.
			accumulator += voltageValue * voltageValue;	
			
			// 2. Check for accumulator overflow or underflow and prevent them if they occur.
			accumulator = preventAccumulatorOverflowOrUnderflow(accumulator, windowSize * vREF * vREF);
			
			// 3. Update the buffer.
			buffer[sampleCounter] = voltageValue;
		}
		// Otherwise if the buffer has been filled...
		else
		{
			// 1. Update the accumalor.
			accumulator = accumulator - (buffer[sampleCounter] * buffer[sampleCounter]) + (voltageValue * voltageValue);
			
			// 2. Check for accumulator overflow or underflow and prevent them if they occur.
			accumulator = preventAccumulatorOverflowOrUnderflow(accumulator, windowSize * vREF * vREF);
			
			// 3. Update the buffer.
			buffer[sampleCounter] = voltageValue;
		}
		
		// Increment the sample counter.
		sampleCounter++;
		
		// If the sample counter has reached the window size, set the bufferFull flag to 1 and reset the counter.
		if(sampleCounter == windowSize)
		{
			bufferFull = 1;
			sampleCounter = 0;
		}
		
		// Every N ms, where N is the window size, compute a new RMS voltage and update the array that contains the digits to be displayed.
		if(sampleCounter == 0)
		{
			vRMS = sqrt(accumulator / windowSize);
			printf("RMS Voltage: %f\n", vRMS);
			
			// The RMS voltage has the form A.BCDEF...
			// Retrive digit A and put it in digitArray[0].
			digitArray[0] = (int) vRMS;
			
			// Retrive digit B and put it in digitArray[1].
			digitArray[1] = ((int)(vRMS * 10)) % 10;
			
			// Retrive digit C and put it in digitArray[2].
			digitArray[2] = ((int)(vRMS * 100)) % 10;
		} 
  }
	// TIMER 3: This gets called every 5 ms.
  else if(htim == &htim3)
  {
		// Generate a 50 Hz square wave with an amplitude of 3V.
    squareWaveCounter = (squareWaveCounter + 1) % 2;
    if(squareWaveCounter == 0)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
    }
		
    // Refresh the segment display and update the digit counter.
    displayNumberOnTrgtDigit(digitArray[digitCounter], (digitCounter + 1) );
    digitCounter = (digitCounter + 1) % 3;
  }
}

// This function prevents unexpected spike in input value.
int preventUnexpectedSpike(int currentADCValue)
{
		if(currentADCValue < 0)
		{
			return 0;
		}
		else if(currentADCValue > 255)
		{
			return 255;
		}
		else
		{
			return currentADCValue;
		}
}

// This function prevents accumulator overflow/underflow.
double preventAccumulatorOverflowOrUnderflow(double accumulator, double maxValue)
{
	if(accumulator < 0)
	{
		return 0;
	}
	else if(accumulator > maxValue)
	{
		return maxValue;
	}
	else
	{
		return accumulator;
	}
}

// This function selects the provided number to be lit up (selects the right segments of the 7-segment display).
void displayNumberOn7Segment(int number) 
{   
    switch(number) 
    {
      case 0:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); //G
        break;
      
      case 1:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); //G
        break;
        
      case 2:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); //G
        break;
      
      case 3:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); //G
        break;
      
      case 4:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); //G
        break;
      
      case 5:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); //G
        break;
      
      case 6:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); //G
        break;
      
      case 7:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); //G
        break;
      
      case 8:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); //G
        break;
      
      case 9:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //A
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //B
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //C
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //D
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //E
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); //F
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); //G
        break;
    }
}
  
// This function selects the provided digit to be lit up.
void selectTrgt7SegmentDisplayDigit(int trgtDigit)
{
  switch(trgtDigit)
  {
    case 1:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
    
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // Light up Decimal point
      break;
    
    case 2:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); 
    
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Shut down Decimal point
      break;
    
    case 3:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); 
    
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Shut down Decimal point
      break;
  }
}

// This function lits up the provided number on the provided digit.
void displayNumberOnTrgtDigit(int number, int trgtDigit)
{ 
  selectTrgt7SegmentDisplayDigit(trgtDigit);
  displayNumberOn7Segment(number);
}

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
