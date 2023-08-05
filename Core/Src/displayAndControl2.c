/*
 * displayAndControl.c
 *
 *  Created on: 19 paź 2022
 *      Author: monio
 */
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "GFX.h"

struct pidStruct{
	double doubleP;
	double doubleI;
	double doubleD;

};



void  sending(uint32_t torque){ // sending informarion to PC about encoder UP position and Torque

		                        char itoaBuffer[6];
				            	char F[]={"F"};
				            	char P[]={"P"};
				            	char L[]={"L"};
				            	send_string(F);
				            	int encoderCounterUp3 = htim3.Instance->CNT;
				            	itoa(encoderCounterUp3,itoaBuffer,10);
				                send_string(itoaBuffer);

				                send_string(P);
				                itoa(torque,itoaBuffer,10);
				                send_string(itoaBuffer);
				                send_string(L);
}

void firstScreenOled() // frames of first stage
{

	unsigned char  EncDownMessage[] = {"ENC DN : "};
	unsigned char  EncUpMessage[] = {"ENC UP : "};
	unsigned char  PWMMessage[] = {"PWM : "};
	unsigned char  MODEMessage[] = {"MODE: "};


		      SSD1306_draw_fast_hline(0,43,200,WHITE);
		      GFX_draw_string(0, 33, EncUpMessage , WHITE,BLACK, 1, 1);

		      SSD1306_draw_fast_hline(0,32,200,WHITE);
		      GFX_draw_string(0, 22, EncDownMessage, WHITE,BLACK, 1, 1);

		      SSD1306_draw_fast_hline(0,21,200,WHITE);
		      GFX_draw_string(0, 11,PWMMessage , WHITE,BLACK, 1, 1);

		      SSD1306_draw_fast_hline(0,10,200,WHITE);
		      GFX_draw_string(0, 0, MODEMessage, WHITE,BLACK, 1, 1);

			  SSD1306_display_repaint();
}



void encoderUpstageControl() // info about upper encoder
{

	 int encoderCounterUp;
	 encoderCounterUp = htim3.Instance->CNT;
	 char itoaBuffer[20];
	 unsigned char  blank[] = {"    "};

	 if (encoderCounterUp >=1000)
			     		      {
			     		    	  itoa(encoderCounterUp,itoaBuffer,10);
			     		    	  GFX_draw_string(60, 33, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
			     		    	  SSD1306_display_repaint();

			     		      }else
			     		      {
			     		    	   GFX_draw_string(60, 33,( unsigned char*)blank, WHITE,BLACK, 1, 1);
			     		    	   itoa(encoderCounterUp,itoaBuffer,10);
			     		    	   GFX_draw_string(60, 33, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
			     		    	   SSD1306_display_repaint();
			     		      }
}

void PIDstageControl(struct pidStruct pid) // info about chosen pids
{
	char itoaBuffer[20];
	unsigned char  P[] = {"P:"};
	unsigned char  I[] = {"I:"};
	unsigned char  D[] = {"D:"};
	GFX_draw_string(95, 33,( unsigned char*)P, WHITE,BLACK, 1, 1);
	itoa(pid.doubleP,itoaBuffer,10);
	GFX_draw_string(110, 33, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
	SSD1306_display_repaint();

	 GFX_draw_string(95, 22,( unsigned char*)I, WHITE,BLACK, 1, 1);
	 itoa(pid.doubleI,itoaBuffer,10);
	 GFX_draw_string(110, 22, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
	 SSD1306_display_repaint();

	 GFX_draw_string(95, 11,( unsigned char*)D, WHITE,BLACK, 1, 1);
	 itoa(pid.doubleD,itoaBuffer,10);
	 GFX_draw_string(110, 11, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
	 SSD1306_display_repaint();


}

void encoderDownstageControl() // info about counting down encoder
{

	int encoderCounterDown;
	encoderCounterDown = htim4.Instance->CNT;
    unsigned char  blank[] = {"    "};
	char itoaBuffer[20];
	   if (encoderCounterDown >=1000)
	 	 {
	 		    	  itoa(encoderCounterDown,itoaBuffer,10);
	 		    	  GFX_draw_string(60, 22, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
	 		    	  SSD1306_display_repaint();

	 	 }else
	 	 {
	 		  		  GFX_draw_string(60, 22,( unsigned char*)blank, WHITE,BLACK, 1, 1);
	 		  		  itoa(encoderCounterDown,itoaBuffer,10);
	 		  		  GFX_draw_string(60, 22, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
	 		  		  SSD1306_display_repaint();
	 	  }



	}



volatile void oledFirstStageControl(volatile uint8_t *sendFrame) // info about frame from PC
{


	    volatile uint8_t frame[13];

	    for (int i =0;i<13;i++)
	      {

	          frame[i]=sendFrame[i];
	      }


	    unsigned char* OledString;
	    OledString =( unsigned char*)frame;
        GFX_draw_string(25, 25, OledString, WHITE,BLACK, 0, 0);
		GFX_draw_string(0, 45, OledString, WHITE,BLACK, 1, 1);
		SSD1306_display_repaint();



}
void infoOled(unsigned char * itoaBuffer){ // your own information

	 GFX_draw_string(70, 11, itoaBuffer, WHITE,BLACK, 1, 1);
	 SSD1306_display_repaint();
}

void pwmOled(uint32_t PWM) // info about actual torque
{
	 char itoaBuffer[20];
	 itoa(PWM,itoaBuffer,10);
	 GFX_draw_string(32, 11, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
	 SSD1306_display_repaint();
}
void startOled(){ // info about start mode
	unsigned char messageStart[] = { "MODE: START       "};
	GFX_draw_string(0, 0,messageStart, WHITE,BLACK, 1, 1);
	SSD1306_display_repaint();
}


void stopOled(){ // info about stop mode
	unsigned char  stopMessage[] = {"MODE: STOP        "};
	GFX_draw_string(0, 0, stopMessage, WHITE,BLACK, 1, 1);
	SSD1306_display_repaint();
}


void manualLeftOled(){ // info about left move


	unsigned char messageManualLeft[] = { "MODE: MANUAL LEFT      "};
    GFX_draw_string(0, 0, messageManualLeft, WHITE,BLACK, 1, 1);
	SSD1306_display_repaint();
		      }

void manualrightOled(){ // info about right move

	unsigned char messageManualRight[] = { "MODE: MANUAL RIGHT      "};
	GFX_draw_string(0, 0, messageManualRight, WHITE,BLACK, 1, 1);
	SSD1306_display_repaint();
		  		      }
void initWork()// information on GPIO led about initialization
{
	int i=0;
	do{

		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		HAL_Delay(300);
		i++;
	}while(i<10);


}

void errorCheck(uint8_t communicationFrame[13])
{

	if(communicationFrame[0] != 1 && communicationFrame[13]!=1)
	{int i=0;
		do{

			HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin );
			HAL_Delay(500);
			i++;
		}while(i<10);


	}
}

