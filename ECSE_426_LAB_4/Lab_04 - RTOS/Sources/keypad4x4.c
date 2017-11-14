#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "keypad4x4.h"
#include <stdio.h>

const int keypadMap[4][4] =
{
	{1,  2,  3,  20}, // 20 = A
	{4,  5,  6,  21},	// 21 = B
	{7,  8,  9,  22},	// 22 = C
	{0, 10, 11,  23}  // 23 = D
};

GPIO_InitTypeDef COLS_GPIO_init;
GPIO_InitTypeDef ROWS_GPIO_init;

// THIS FUNCTION SETS THE COLUMNS AS INPUT.
void setColumnsAsInput(void)
{
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	// COLUMN = INPUT.
	COLS_GPIO_init.Pin   = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
	COLS_GPIO_init.Mode  = GPIO_MODE_INPUT;
	COLS_GPIO_init.Pull  = GPIO_PULLUP;
	COLS_GPIO_init.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE, &COLS_GPIO_init);
	 
	// ROW = OUTPUT.
	ROWS_GPIO_init.Pin   = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	ROWS_GPIO_init.Mode  = GPIO_MODE_OUTPUT_PP;
	ROWS_GPIO_init.Pull  = GPIO_PULLDOWN;
	ROWS_GPIO_init.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE, &ROWS_GPIO_init);
}

// THIS FUNCTION SETS THE ROWS AS INPUT.
void setRowsAsInput(void)
{
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	// COLUMN = OUTPUT.
	COLS_GPIO_init.Pin   = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
	COLS_GPIO_init.Mode  = GPIO_MODE_OUTPUT_PP;
	COLS_GPIO_init.Pull  = GPIO_PULLDOWN;
	COLS_GPIO_init.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE, &COLS_GPIO_init);
	
	// ROW = INPUT.
	ROWS_GPIO_init.Pin   = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	ROWS_GPIO_init.Mode  = GPIO_MODE_INPUT;
	ROWS_GPIO_init.Pull  = GPIO_PULLUP;
	ROWS_GPIO_init.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE, &ROWS_GPIO_init);
}

// THIS FUNCTION RETURNS THE COLUMN NUMBER FOR THE KEY THAT IS PRESSED.
int getColumnOfKeyPressed(void)
{
	setColumnsAsInput();
	
	// LEFTMOST TO RIGHTMOST COLUMN: PE7, PE8, PE9, PE10.
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_RESET)
	{
		return 0;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_RESET)
	{
		return 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET)
	{
		return 2;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == GPIO_PIN_RESET)
	{
		return -1;
	}
	else
	{
		return -1;
	}
}

// THIS FUNCTION RETURNS THE ROW NUMBER FOR THE KEY THAT IS PRESSED.
int getRowOfKeyPressed(void)
{
	setRowsAsInput();
	
	// TOPMOST TO BOTTOMMOST ROW: PE11, PE12, PE13, PE14.
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_RESET)
	{
		return 0;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == GPIO_PIN_RESET)
	{
		return 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_RESET)
	{
		return 2;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_RESET)
	{
		return 3;
	}
	else
	{
		return -1;
	}
}

// THIS FUNCTION RETURNS A NUMBER REPRESENTING WHICH KEY IS PRESSED (SEE KEYPAD_MAP ABOVE).
int getKeyPressed(void)
{
	int key;
	int column = getColumnOfKeyPressed();
	int row = getRowOfKeyPressed();
	
	if((row == -1) || (column == -1))
	{
		key = -1;
	}
	else
	{
		key = keypadMap[row][column];
	}
	
	return key;
}
