#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "fsm.h"
#include "tim.h"
#include "global_variables.h"
#include <stdio.h>

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
				
				//HAL_TIM_Base_MspDeInit(&htim2);
				//HAL_TIM_Base_MspDeInit(&htim4);
				//HAL_TIM_PWM_MspDeInit(&htim4);
				//deInit_External_Trigger();
				
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
