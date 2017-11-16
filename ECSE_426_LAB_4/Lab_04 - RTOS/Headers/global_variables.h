#ifndef _GLOBAL_VARIABLES_H
#define _GLOBAL_VARIABLES_H

// THIS FLAG IS USED BY THE ACCELEROMETER INTERRUPT.
extern int flagForAccelerometer;

// THIS FLAG IS USED BY THE KEYPAD/7-SEGMENT DISPLAY INTERRUPT.
extern int flagForKeypad;

// THIS IS THE BUFFER FOR THE INPUT OF THE ACCELEROMETER SENSOR.
extern float inputBuffer[3];

// THESE VARIABLES STORE THE CURRENT RAW VALUES FROM THE ACCELEROMETER SENSOR.
extern float accX, accY, accZ;

// THESE CONSTANTS ARE USED TO CALIBRATE THE VALUES OF THE ACCELEROMETER SENSOR.
extern float	acc11;
extern float	acc12;
extern float	acc13;
extern float	acc10;
extern float	acc21;
extern float	acc22;
extern float	acc23;
extern float	acc20;
extern float	acc31;
extern float	acc32;
extern float	acc33;
extern float	acc30;

// THESE VARIABLES STORE THE CURRENT CALIBRATED VALUES.
extern float normalizedAccX, normalizedAccY, normalizedAccZ;

// THESE ARRAYS KEEP TRACK OF THE LAST THREE UNFILTERED ACCELEROMETER READINGS FOR A PARTICULAR AXIS: {x[n], x[n-1], x[n-2]}.
extern float unfilteredAccX[3];
extern float unfilteredAccY[3];
extern float unfilteredAccZ[3];

// THESE ARRAYS KEEP TRACK OF THE LAST TWO FILTERED ACCELEROMETER VALUES FOR A PARTICULAR AXIS: {y[n-1], y[n-2]}.
extern float filteredAccX[2];
extern float filteredAccY[2];
extern float filteredAccZ[2];

// THESE CONSTANTS ARE THE FILTER COEFFICIENTS.
extern float b0;
extern float b1;
extern float b2;
extern float a1;
extern float a2;

// THESE VARIABLES STORE THE CURRENT FILTERED AND CALIBRATED VALUES.
extern float finalAccX, finalAccY, finalAccZ;

// THESE VARIABLES STORE THE CURRENT ROLL AND PITCH ANGLES.
extern double rollAngle;
extern double pitchAngle;

// THESE VARIABLES STORE THE TARGET ROLL AND PITCH ANGLES (ENTERED ON THE KEYPAD).
extern int targetRollAngle;
extern int targetPitchAngle;

// THIS ARRAY KEEPS TRACK OF THE DIGIT TO BE DISPLAYED ON THE 7-SEGMENT DISPLAY.
extern int digitArray[3];

// THIS VARIABLE KEEPS TRACK OF WHICH DIGIT OF THE LED DISPLAY SHOULD BE LIGHT UP.
extern int digitCounter;

// THIS COUNTER IS USED TO REFRESH THE VALUE OF THE 7-SEGMENT DISPLAY EVERY 1 MS.
extern int refreshCounter;

// THIS VARIABLE KEEPS TRACK OF THE CURRENT KEY PRESSED.
extern int currentKeyPressed;

// THIS VARIABLE KEEPS TRACK OF HOW LONG A KEY IS PRESSED.
extern int keyPressedCounter;

// THIS VARIABLE KEEPS TRACK OF THE STATE.
// STATE 1: ENTER ROLL ANGLE.
// STATE 2: ENTER PITCH ANGLE.
// STATE 3: OPERATION MODE.
// STATE 4: SLEEP MODE.
extern int FSM_state;

// THIS VARIABLE KEEPS TRACK OF WHICH STATE TO GO BACK TO AFTER WAKING UP FROM SLEEP MODE.
extern int wake_up_state;

// THIS BOOLEAN VARIABLE KEEPS TRACK OF WHETHER THE ROLL OR PITCH ANGLE SHOULD BE DISPLAYED IN OPERATION MODE.
extern int isRollAngleDisplayed;

#endif
