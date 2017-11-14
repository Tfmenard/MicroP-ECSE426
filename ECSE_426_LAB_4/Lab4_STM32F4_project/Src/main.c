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
#include "main.h"

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
int targetRollAngle  = 0;
int targetPitchAngle = 0;

// THIS ARRAY KEEPS TRACK OF THE DIGIT TO BE DISPLAYED ON THE 7-SEGMENT DISPLAY.
int digitArray[3] = {0, 0, 0};

// THIS VARIABLE KEEPS TRACK OF WHICH DIGIT OF THE LED DISPLAY SHOULD BE LIGHT UP.
int digitCounter = 0;

// THIS COUNTER IS USED TO REFRESH THE VALUE OF THE 7-SEGMENT DISPLAY EVERY 1 MS.
int refreshCounter = 0;

// THIS VARIABLE KEEPS TRACK OF THE CURRENT KEY PRESSED.
int currentKeyPressed = -1;

// THIS VARIABLE KEEPS TRACK OF HOW LONG A KEY IS PRESSED.
int keyPressedCounter = 0;

// THIS VARIABLE KEEPS TRACK OF THE STATE.
// STATE 1: ENTER ROLL ANGLE.
// STATE 2: ENTER PITCH ANGLE.
// STATE 3: OPERATION MODE.
// STATE 4: SLEEP MODE.
int FSM_state = 1;

// THIS VARIABLE KEEPS TRACK OF WHICH STATE TO GO BACK TO AFTER WAKING UP FROM SLEEP MODE.
int wake_up_state = 1;

// THIS BOOLEAN VARIABLE KEEPS TRACK OF WHETHER THE ROLL OR PITCH ANGLE SHOULD BE DISPLAYED IN OPERATION MODE.
int isRollAngleDisplayed = 1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void initializeACC			(void);
int SysTickCount;

int main(void)
{
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

	printf("ENTER ROLL ANGLE! \n\n");
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
	if(FSM_state == 3)
	{
		updatePulse(deltaRoll,  TIM_CHANNEL_3, &htim4);
		updatePulse(deltaRoll,  TIM_CHANNEL_1, &htim4);
		updatePulse(deltaPitch, TIM_CHANNEL_4, &htim4);
		updatePulse(deltaPitch, TIM_CHANNEL_2, &htim4);
	}
	else
	{
		updatePulse(0,  TIM_CHANNEL_3, &htim4);
		updatePulse(0,  TIM_CHANNEL_1, &htim4);
		updatePulse(0, TIM_CHANNEL_4, &htim4);
		updatePulse(0, TIM_CHANNEL_2, &htim4);
	}

	// UPDATE THE DIGIT ARRAY TO BE DISPLAYED ON THE 7-SEGMENT DISPLAY.
	refreshCounter++;
	if(refreshCounter == 50)
	{
		switch(FSM_state)
		{
			// STATE 1: ENTER ROLL ANGLE.
			case 1:
				digitArray[0] = (targetRollAngle / 100) % 10;
				digitArray[1] = (targetRollAngle / 10)  % 10;
				digitArray[2] = (targetRollAngle / 1)   % 10;
				break;
	
			// STATE 2: ENTER PITCH ANGLE.
			case 2:
				digitArray[0] = (targetPitchAngle / 100) % 10;
				digitArray[1] = (targetPitchAngle / 10)  % 10;
				digitArray[2] = (targetPitchAngle / 1)   % 10;
				break;
		
			// STATE 3: OPERATION MODE.
			case 3:
				if(isRollAngleDisplayed == 1)
				{
					digitArray[0] = ((int)(rollAngle / 100)) % 10;
					digitArray[1] = ((int)(rollAngle / 10))  % 10;
					digitArray[2] = ((int)(rollAngle / 1))   % 10;
				}
				else
				{
					digitArray[0] = ((int)(pitchAngle / 100)) % 10;
					digitArray[1] = ((int)(pitchAngle / 10))  % 10;
					digitArray[2] = ((int)(pitchAngle / 1))   % 10;
				}
				break;
		
			// STATE 4: SLEEP MODE.
			case 4:
				break;
		}
		refreshCounter = 0;
	}
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
	// THE INTERRUPT OF TIMER 2 GETS CALLED EVERY 7 MS.
  if(htim == &htim2)
  {
		int key = getKeyPressed();
		
		if(key != -1)
		{
			currentKeyPressed = key;
			if(keyPressedCounter < 1000)
			{
				keyPressedCounter++;
			}
		}
		else if(key != currentKeyPressed && key == -1)
		{
			printf("The key %d was pressed for %d ms. \n", currentKeyPressed, keyPressedCounter * 7);
			fsmEvent(currentKeyPressed, keyPressedCounter);
			currentKeyPressed = -1;
			keyPressedCounter = 0;
		}
		
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
}

// THIS FUNCTION SELECTS THE PROVIDED NUMBER TO BE LIGHT UP (SELECTS THE RIGHT SEGMENTS OF THE 7-SEGMENT DISPLAY).
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
  
// THIS FUNCTION SELECTS THE PROVIDED DIGIT TO BE LIGHT UP.
void selectTrgt7SegmentDisplayDigit(int trgtDigit)
{
  switch(trgtDigit)
  {
		// First digit.
    case 1:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Shut down Decimal point
      break;
    
		// Second digit.
    case 2:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); 
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Shut down Decimal point
      break;
    
		// Third digit.
    case 3:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Shut down Decimal point
      break;
		
		// No digit.
    case 0:
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Shut down Decimal point
      break;
  }
}

// THIS FUNCTION LIGHTS UP THE PROVIDED NUMBER ON THE PROVIDED DIGIT.
void displayNumberOnTrgtDigit(int number, int trgtDigit)
{ 
  selectTrgt7SegmentDisplayDigit(trgtDigit);
  displayNumberOn7Segment(number);
}

// THIS FUNCTION HANDLES THE FINITE STATE MACHINE.
void fsmEvent(int keyPressed, int keyPressedCounter)
{
	switch(FSM_state)
	{
		// STATE 1: ENTER ROLL ANGLE.
		case 1:
			if(keyPressed == 0 || keyPressed == 1 || keyPressed == 2 || keyPressed == 3 || keyPressed == 4 || keyPressed == 5 || keyPressed == 6 || keyPressed == 7 || keyPressed == 8 || keyPressed == 9)
			{
				targetRollAngle = (targetRollAngle * 10) + keyPressed;
				printf("TARGET ROLL ANGLE: %d \n\n", targetRollAngle);
			}
			else if(keyPressed == 10)
			{
				if(targetRollAngle >= 0 && targetRollAngle <= 180)
				{
					FSM_state = 2;
					printf("VALID TARGET ROLL ANGLE: %d \n", targetRollAngle);
					printf("ENTER PITCH ANGLE! \n\n");
				}
				else
				{
					printf("INVALID TARGET ROLL ANGLE: %d \n", targetRollAngle);
					printf("ENTER ROLL ANGLE BETWEEN 0 AND 180! \n\n");
					targetRollAngle = 0;
				}
			}
			else if(keyPressed == 11 && keyPressedCounter > 428)
			{
				FSM_state = 4;
				wake_up_state = 1;
				printf("SLEEP MODE. GOOD NIGHT! \n\n");
			}
			else if(keyPressed == 11 && keyPressedCounter > 285)
			{
				targetRollAngle = 0;
				targetPitchAngle = 0;
				FSM_state = 1;
				printf("RESET. ENTER ROLL ANGLE! \n\n");
			}
			else if(keyPressed == 11)
			{
				targetRollAngle = targetRollAngle / 10;
				printf("[BACKSPACE] TARGET ROLL ANGLE: %d \n\n", targetRollAngle);
			}
			else
			{
				// Nothing should happen.
			}
			break;
	
		// STATE 2: ENTER PITCH ANGLE.
		case 2:
			if(keyPressed == 0 || keyPressed == 1 || keyPressed == 2 || keyPressed == 3 || keyPressed == 4 || keyPressed == 5 || keyPressed == 6 || keyPressed == 7 || keyPressed == 8 || keyPressed == 9)
			{
				targetPitchAngle = (targetPitchAngle * 10) + keyPressed;
				printf("TARGET PITCH ANGLE: %d \n\n", targetPitchAngle);
			}
			else if(keyPressed == 10)
			{
				if(targetPitchAngle >= 0 && targetPitchAngle <= 180)
				{
					FSM_state = 3;
					printf("VALID TARGET PITCH ANGLE: %d \n", targetPitchAngle);
					printf("PLAY THE GAME! \n\n");
				}
				else
				{
					printf("INVALID TARGET PITCH ANGLE: %d \n", targetPitchAngle);
					printf("ENTER PITCH ANGLE BETWEEN 0 AND 180! \n\n");
					targetPitchAngle = 0;
				}
			}
			else if(keyPressed == 11 && keyPressedCounter > 428)
			{
				FSM_state = 4;
				wake_up_state = 2;
				printf("SLEEP MODE. GOOD NIGHT! \n\n");
			}
			else if(keyPressed == 11 && keyPressedCounter > 285)
			{
				targetRollAngle = 0;
				targetPitchAngle = 0;
				FSM_state = 1;
				printf("RESET. ENTER ROLL ANGLE! \n\n");
			}
			else if(keyPressed == 11)
			{
				targetPitchAngle = targetPitchAngle / 10;
				printf("[BACKSPACE] TARGET PITCH ANGLE: %d \n\n", targetPitchAngle);
			}
			else
			{
				// Nothing should happen.
			}
			break;
		
		// STATE 3: OPERATION MODE.
		case 3:
			if(keyPressed == 1)
			{
				isRollAngleDisplayed = 1;
				printf("DISPLAY ROLL ANGLE! \n\n");
			}
			else if(keyPressed == 2)
			{
				isRollAngleDisplayed = 0;
				printf("DISPLAY PITCH ANGLE! \n\n");
			}
			else if(keyPressed == 11 && keyPressedCounter > 428)
			{
				FSM_state = 4;
				wake_up_state = 3;
				printf("SLEEP MODE. GOOD NIGHT! \n\n");
			}
			else if(keyPressed == 11 && keyPressedCounter > 285)
			{
				targetRollAngle = 0;
				targetPitchAngle = 0;
				FSM_state = 1;
				printf("RESET. ENTER ROLL ANGLE! \n\n");
			}
			else
			{
				// Nothing should happen.
			}
			break;
		
		// STATE 4: SLEEP MODE.
		case 4:
			if(keyPressed == 10 && keyPressedCounter > 428)
			{
				FSM_state = wake_up_state;
				printf("WAKING UP. HELLO! \n\n");
			}
			else
			{
				// Nothing should happen.
			}
			break;
	}
	return;
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
