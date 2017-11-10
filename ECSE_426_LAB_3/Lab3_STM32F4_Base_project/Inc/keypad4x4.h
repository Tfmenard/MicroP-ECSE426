#ifndef _KEYPAD_H
#define _KEYPAD_H

#include "stm32f4xx_hal.h"
#include "gpio.h"

void setColumnsAsInput(void);
void setRowsAsInput(void);
int getColumnOfKeyPressed(void);
int getRowOfKeyPressed(void);
int getKeyPressed(void);

#endif
