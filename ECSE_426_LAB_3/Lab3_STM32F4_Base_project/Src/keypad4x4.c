#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "keypad4x4.h"
#include <stdio.h>


const int keypadMap[4][4] = {
	{1, 2, 3, 20},  //20 =  A
	{4, 5, 6, 21},	//21 = B
	{7, 8, 9, 22},	//22 = C
	{11, 0, 12, 23} //23 = D
};


GPIO_InitTypeDef COLS_GPIO_init;
GPIO_InitTypeDef ROWS_GPIO_init;

// Set columns as inputs and rows as outputs to read the read values on columns
void setColsAsInputs(void)
{
	// Set the columns as inputs and initialize struct
	__HAL_RCC_GPIOE_CLK_ENABLE();
	COLS_GPIO_init.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10; 
	COLS_GPIO_init.Mode = GPIO_MODE_INPUT; // Configure column as input
	COLS_GPIO_init.Pull = GPIO_PULLDOWN; // Pin value set to high
	COLS_GPIO_init.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOE,&COLS_GPIO_init); 
	 
	// Set the rows as outputs and initialize struct
	__HAL_RCC_GPIOE_CLK_ENABLE(); 
	ROWS_GPIO_init.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14; 
	ROWS_GPIO_init.Mode = GPIO_MODE_OUTPUT_PP; // Configure row as output
	ROWS_GPIO_init.Pull = GPIO_NOPULL; // Pin value is set to low
	ROWS_GPIO_init.Speed = GPIO_SPEED_LOW; 
	HAL_GPIO_Init(GPIOE,&ROWS_GPIO_init);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);
}


// Set rows as inputs and rows as outputs to read the read values on columns
void setRowsAsInputs(void)
{
	// Set the rows as inputs and initialize struct
	__HAL_RCC_GPIOE_CLK_ENABLE();
	COLS_GPIO_init.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
	COLS_GPIO_init.Mode = GPIO_MODE_OUTPUT_PP; // Column is now configured to output
	COLS_GPIO_init.Pull = GPIO_NOPULL; // Pin value set to low
	COLS_GPIO_init.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOE,&COLS_GPIO_init); 
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
	
	// Set the columns as outputs and initialize struct
	__HAL_RCC_GPIOE_CLK_ENABLE(); 
	ROWS_GPIO_init.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14; 
	ROWS_GPIO_init.Mode = GPIO_MODE_INPUT; // Row is now configured to input
	ROWS_GPIO_init.Pull = GPIO_PULLDOWN; // Pin value set to high
	ROWS_GPIO_init.Speed = GPIO_SPEED_LOW; 
	HAL_GPIO_Init(GPIOE,&ROWS_GPIO_init); 
}

// Function to return the column pressed
int getPressedColumn(void)
{
	// The columns are inputs and are set to high
	setColsAsInputs();
	
	// Return the value of column pressed (pin of column pressed is going to be low)
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET)
	{
		return 0;
	}
	else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_SET)
	{
		return 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) == GPIO_PIN_SET)
	{
		return 2;
	}
	
	else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_SET)
	{
		return 2;
	}
	else return -1;
}


// Function to return the row pressed
int getPressedRow(void)
{
	// The rows are inputs and are set to high
	setRowsAsInputs();
	
	// Return the value of row pressed (pin of row pressed is going to be low)
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET)
	{
		return 0;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == GPIO_PIN_SET)
	{
		return 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)
	{
		return 2;
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_SET)
	{
		return 3;
	}
	else return -1;
}

// Function to return the key pressed
int getPressedKey(int row, int column)
{
	int key;

	
	if( (row == -1) || (column == -1)){ // If key is not pressed
		key = -1;
	}else{
		key = keypadMap[row][column];
	}
	return key;
}


//Function to reset input angle operation
int resetKey(void)
{
	int key = getPressedKey(1,2);
	if(key == 11)
	{
		int counter = 0;
		int reset_counter = 0;
		
		while(counter < 100000)
		{
			if(getPressedKey(1,2) == 11)
			{
				reset_counter++;
			}
			counter++;
		}
		
		if(reset_counter > 50000)
		{
			return 1;
		} 
		else 
		{
			return 0;
		}
	} 
	else 
	{
		return 0;
	}
}
