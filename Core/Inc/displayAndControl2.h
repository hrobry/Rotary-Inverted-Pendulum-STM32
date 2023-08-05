/*
 * displayAndControl2.h
 *
 *  Created on: Mar 11, 2023
 *      Author: monio
 */

#ifndef INC_DISPLAYANDCONTROL2_H_
#define INC_DISPLAYANDCONTROL2_H_





#include "gpio.h"
#include "GFX.h"

 struct pidy{
	double doubleP;
	double doubleI;
	double doubleD;

};

void initWork();
void  sending(uint32_t *torque);
void firstScreenOled();
void encoderDownstageControl();
void encoderUpstageControl();
void PIDstageControl(struct pidy pid);
void manualLeftOled();
void manualrightOled();
void stopOled();
void startOled();
void pwmOled(uint32_t PWM);
void errorCheck(uint8_t communicationFrame[13]);
void infoOled(unsigned char * itoaBuffer);
volatile void oledFirstStageControl(volatile uint8_t sendFrame[13]);
#endif /* INC_DISPLAYANDCONTROL_H_ */
