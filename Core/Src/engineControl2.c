/*
 * engineControl2.c
 *
 *  Created on: 12 mar 2023
 *      Author: monio
 */


/*
 * engineControl.c
 *
 *  Created on: 19 paź 2022
 *      Author: monio
 */



#include <stdbool.h>
#include "displayAndControl2.h"
#include "tim.h"
#include "usart.h"

static uint32_t direction; // direction of engine working

static uint32_t outputPID; // counted of value PWM

struct pidStruct{
	double doubleP;
	double doubleI;
	double doubleD;

};

double _maxi = 1024;
double _mini=0;
static double _pre_error;
double _integral=0;
double derivative;
uint32_t setTime = 100;
uint32_t now,timechange;
static uint32_t lasttime;
double error;
double Pout,Iout,Dout;
double Doutput;


void send_strings(char* s) // function of sending data to PC
{
	HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}



 uint32_t calculatePID( int input, int setpoint ,struct pidStruct pidValues ) //engineControl calculate PWM value from pid
{

    	_pre_error=0;
	    now = HAL_GetTick (); // get current time in microseconds
		timechange = (now - lasttime);
		if(timechange >=setTime)
		{
			     error = setpoint - input;
			     Pout = pidValues.doubleP * error; // proportional part calculate
			     _integral += error * timechange;
			     Iout = pidValues.doubleI * _integral; // integral part calculate
			     derivative = (error - _pre_error) / timechange;
			     Dout = pidValues.doubleD * derivative; // differential part calculate
			     Doutput = Pout + Iout + Dout;
			    	if (Doutput<0)
							{
								direction = 1; // direction of motor
							}else if(Doutput>0){
								direction = 0;
							}
			    if( Doutput < _mini )
			        {Doutput =Doutput * -1;} // if output value is lower than 0 multiply by -1
			    else if( Doutput > _maxi )
			    { outputPID = _maxi;}
			    outputPID=(uint32_t)Doutput;
			    _pre_error = error;
				lasttime = now;
		}
 return outputPID;
}

 struct pidStruct pidsConvert( char *pidsFrames, uint8_t *pidsFrame)//engine_control converting uint8_t PID data from PC to char values
{
	struct pidStruct outputPid;
	 char *arayPidP;
	 char *arayPidI;
	 char *arayPidD;

for (int i = 0 ; i<35;i++){
	memcpy( &pidsFrames[i],(char *)&pidsFrame[i+13] ,1); // copy all values from uint8_t array to char array

	}

char *ptr=pidsFrames;
char *p;
char korektor[] = " ";
arayPidP= strtok_r(ptr, korektor, &ptr); // delete spaces
outputPid.doubleP =strtod(arayPidP, &p); // convert from char to double
arayPidI=strtok_r(ptr, korektor, &ptr);
outputPid.doubleI =strtod(arayPidI, &p);
arayPidD=strtok_r(ptr, korektor, &ptr);
outputPid.doubleD =strtod(arayPidD, &p);
return outputPid;
}


volatile void engineControl( uint8_t * sendFrame  , uint32_t *torque, int *controlOfComm)//engine_control
{
  //  calculatePID( encoderCounterUp2, 595 ,9.5,0.000004,3.5);

	 static char pidsFrames[35];
	 static struct pidStruct convertedPid;
	 static int encoderCounterUp2=0;
	 static int ondrive = 0;
     static uint32_t out=0;


	if (sendFrame[4] == 49&&sendFrame[5] == 48 && sendFrame[8] == 48) // start of working
	{

		startOled(); // paint start mode

        encoderCounterUp2 = htim3.Instance->CNT;

      if (*controlOfComm== 1){  // if orders from PC arrive
    	  convertedPid =pidsConvert( pidsFrames,sendFrame); // transform PID value again
        *controlOfComm=0;
        }

	    encoderCounterUp2 = htim3.Instance->CNT;
	     if(encoderCounterUp2>580 && encoderCounterUp2 <600){ // if value if inclination is between those two values
		 ondrive =1; // mark ondrive as one
	        }

	    if (encoderCounterUp2>400 && encoderCounterUp2 <800 && ondrive==1) // pendulum is working between those two values
	    {


       out  =calculatePID( encoderCounterUp2, 600 ,convertedPid); // calculate PWM output


      *torque = out; // write PWM output value to torque variable

         if(direction==1)
			{

			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1024);// direction engine control
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);// direction engine control
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, out);
			}else{

			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);// direction engine control
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1024);// direction engine control
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, out); // engine power control
			 }

	     }else{
	    	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);// direction engine control
	    	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);// direction engine control
	         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, out);// engine power control
              ondrive = 0;

	     }

	}



		if (sendFrame[4] == 48&& sendFrame[5] ==49 && sendFrame[8] == 48) // stop control
		{
		             _pre_error=0; // if work is stopped reset all of those variables
			      	 _integral=0;
			      	 derivative=0;
			      	 now=0;
			      	 timechange=0;
			      	 lasttime=0;
			      	 error=0;
			      	convertedPid.doubleP=0;
			      	convertedPid.doubleP=0;
			      	convertedPid.doubleD=0;
			      	Doutput=0;
			        stopOled();

			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);// direction engine control
		     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);// direction engine control
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);// engine power control
		}

		if 	(sendFrame[8] == 49&& sendFrame[7] ==49 && sendFrame[6] == 48)// manual left control
				{
			manualLeftOled();
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1024);// direction engine control
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);// direction engine control
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 400);// engine power control
				}
		if	(sendFrame[8] == 49&& sendFrame[7] ==48 && sendFrame[6] == 49)
				{
			manualrightOled();
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);// direction engine control
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1024);// direction engine control
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 400);// engine power control
				}





		}
