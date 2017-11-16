#ifndef _FSM_H
#define _FSM_H

#include "stm32f4xx_hal.h"
#include "gpio.h"

void fsmEvent(int keyPressed, int keyPressedCounter);

#endif
