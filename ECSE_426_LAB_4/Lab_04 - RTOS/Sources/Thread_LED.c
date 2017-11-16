/*******************************************************************************
  * @file    Thread_LED.c
  * @author  Amirhossein Shahshahani
	* @version V1.2.0
  * @date    10-Nov-2017
  * @brief   This file initializes one LED as an output, implements the LED thread 
  *					 which toggles and LED, and function which creates and starts the thread	
  ******************************************************************************
  */
	
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "stm32f4xx_hal.h"
#include "keypad4x4.h"
#include "7_segment.h"
#include "fsm.h"
#include "math.h"
#include "tim.h"
#include "lis3dsh.h"
#include "global_variables.h"

void Thread_LED_1 (void const *argument);                 // thread function
void Thread_LED_2 (void const *argument);                 // thread function
void Thread_LED_3 (void const *argument);                 // thread function
osThreadId tid_Thread_LED_1;                              // thread id
osThreadId tid_Thread_LED_2;                              // thread id
osThreadId tid_Thread_LED_3;                              // thread 

/*		**** depending on the version of the CMSIS library, you may need to make your threads in different way:   *******

	osThreadDef(Thread_1,Thread_LED, osPriorityNormal, osPriorityNormal, 128);
	osThreadDef(Thread_2, Thread_LED_2, osPriorityNormal, osPriorityNormal, 128);
	osThreadDef(Thread_3, Thread_LED_3, osPriorityNormal, osPriorityNormal, 128);
 
	tid_Thread_LED_1 = osThreadCreate(osThread(Thread_1), NULL);
	tid_Thread_LED_2 = osThreadCreate(osThread(Thread_2), NULL);
	tid_Thread_LED_3 = osThreadCreate(osThread(Thread_3), NULL);

	This is a style when you create a project with CubMx as it uses the newest version

*/

// Following is different format of creating your threads. This project is based on the older CMSIS version.
osThreadDef(Thread_LED_1, osPriorityNormal, 1, 0);
osThreadDef(Thread_LED_2, osPriorityNormal, 1, 0);
osThreadDef(Thread_LED_3, osPriorityNormal, 1, 0);
GPIO_InitTypeDef LED_configuration;

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/
int start_Thread_LED (void)
{
	tid_Thread_LED_1 = osThreadCreate(osThread(Thread_LED_1), NULL);
	tid_Thread_LED_2 = osThreadCreate(osThread(Thread_LED_2), NULL);
	tid_Thread_LED_3 = osThreadCreate(osThread(Thread_LED_3), NULL);
  //if (!tid_Thread_LED_1) return(-1); 
	//else if (!tid_Thread_LED_2) return(-1); 
	//else if (!tid_Thread_LED_3) return(-1); 
  return(0);
}

 /*----------------------------------------------------------------------------
*      Thread  'LED_Thread': Toggles LED
 *---------------------------------------------------------------------------*/
void Thread_LED_1 (void const *argument)
{
	while(1)
	{
		if(flagForKeypad == 1)
		{
			flagForKeypad = 0;
			
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
		}
	}
}

void Thread_LED_2 (void const *argument)
{
	while(1)
	{
		if(flagForAccelerometer == 1)
		{
			flagForAccelerometer = 0;
			
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
				updatePulse(0, TIM_CHANNEL_3, &htim4);
				updatePulse(0, TIM_CHANNEL_1, &htim4);
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
	}
}

void Thread_LED_3 (void const *argument)
{
	while(1)
	{
		osDelay(100);
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	}
}

/*----------------------------------------------------------------------------
 *      Initialize the GPIO associated with the LED
 *---------------------------------------------------------------------------*/
void initializeLED_IO (void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	LED_configuration.Pin		= GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	LED_configuration.Mode 	= GPIO_MODE_OUTPUT_PP;
	LED_configuration.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
	LED_configuration.Pull	= GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &LED_configuration);	
}
/*----------------------------------------------------------------------------
 *      
 *---------------------------------------------------------------------------*/
