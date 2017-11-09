#ifndef _KEYPAD_H
#define _KEYPAD_H

#include "stm32f4xx_hal.h"
#include "gpio.h"

void setColsAsInputs(void);
void setRowsAsInputs(void);
int getPressedColumn(void);
int getPressedRow(void);
int getPressedKey(void);
int interpretKey(void);
int resetkey(void);

#endif
