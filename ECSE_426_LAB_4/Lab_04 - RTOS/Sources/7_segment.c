#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "7_segment.h"
#include <stdio.h>

//THIS FUNCTION SELECTS THE PROVIDED NUMBER TO BE LIGHT UP (SELECTS THE RIGHT SEGMENTS OF THE 7-SEGMENT DISPLAY).
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
