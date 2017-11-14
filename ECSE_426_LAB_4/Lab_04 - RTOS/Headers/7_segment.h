#ifndef _7_SEGMENT_H
#define _7_SEGMENT_H

#include "stm32f4xx_hal.h"
#include "gpio.h"

void displayNumberOn7Segment(int number);
void selectTrgt7SegmentDisplayDigit(int trgtDigit);
void displayNumberOnTrgtDigit(int number, int trgtDigit);

#endif
