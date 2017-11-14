#ifndef __MAIN_H
#define __MAIN_H

void displayNumberOn7Segment(int number);
void selectTrgt7SegmentDisplayDigit(int trgtDigit);
void displayNumberOnTrgtDigit(int number, int trgtDigit);
void fsmEvent(int keyPressed, int keyPressedCounter);

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
