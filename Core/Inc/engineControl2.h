/*
 * engineControl2.h
 *
 *  Created on: 12 mar 2023
 *      Author: monio
 */

#ifndef INC_ENGINECONTROL2_H_
#define INC_ENGINECONTROL2_H_




#include <stdlib.h>
#include <stdbool.h>
#include "gpio.h"

/*
struct calculate{
	uint8_t kierunek;
	uint32_t output;

};
*/
struct controlCommunication{
	uint8_t kier;
	uint32_t calcout;
	int encoderCount;

};


void send_strings(char* s);
uint32_t calculatePID( int input, int setpoint ,struct pidy pid  );
void pidsConvert( char * pidsFrames, uint8_t * pidsFrame);
volatile  void  engineControl( uint8_t * sendFrame , uint32_t *torque, int *controlOfComm);




#endif /* INC_ENGINECONTROL2_H_ */
