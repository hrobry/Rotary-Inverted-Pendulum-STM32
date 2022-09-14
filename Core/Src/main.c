/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "string.h"
#include "GFX.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RxBuf_SIZE 13

#define MainBuf_SIZE 13
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

  // uint8_t communicationFrame  [13] = BIT STARTU= 0xFF , P , I , D , START , STOP , buttonLeft , buttonRight  , manual ,encoderUp,encoderDown,PWM, BIT STOPU = 0x00
 UART_HandleTypeDef huart2;
 DMA_HandleTypeDef hdma_usart2_rx;
 uint8_t P ;
 uint8_t I ;
 uint8_t D ;
 uint8_t START ;
 uint8_t STOP ;
 uint8_t leftButton;
 uint8_t rightButton ;
 uint8_t manual ;
 uint8_t encoderUp=0;
 uint8_t encoderDown=0;
 uint8_t PWM=150;
 uint8_t counter=0;
 uint8_t bufer[44];
 bool kierunek = false;

   char itoaBuffer[20];


 int encoderCounterUp=0;
  int  encoderCounterDown=0;
  char* mesage = "message";

 unsigned char  stopMessage[] = {"MODE: STOP        "};
 unsigned char messageManualRight[] = { "MODE: MANUAL RIGHT      "};
unsigned char messageManualLeft[] = { "MODE: MANUAL LEFT      "};
unsigned char messageStart[] = { "MODE: START       "};

unsigned char  EncDownMessage[] = {"ENC DN : "};
		      unsigned char  EncUpMessage[] = {"ENC UP : "};
		      unsigned char  PWMMessage[] = {"PWM : "};
		      unsigned char  MODEMessage[] = {"MODE: "};

 // P,I,D,Start,STOP,LeftButton,RighrButton,manual,encoderUp,encoderDown,PWM
 uint8_t  communicationFrame [13]={1,1,1,1,0,1,0,0,0,0,0,0,1};





uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
uint8_t sendFrame[13];

unsigned char oledFrame[13];


void oledTransform()
{
	for(int i =0 ; i<13;i++)
	{



	}

}
void frameTransform()
{

	for(int i=0;i<13;i++)
	{
		sendFrame[i]=communicationFrame[i]+48;
	}




}

void errorCheck()
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

 void frameToName(){
		P = communicationFrame [1];
		I = communicationFrame [2];
		D = communicationFrame [3];
		START = communicationFrame [4];
		STOP = communicationFrame [5];
		leftButton = communicationFrame [6];
		rightButton = communicationFrame [7];
		manual = communicationFrame [8];
        communicationFrame [9] = (uint8_t)encoderUp/3;
		communicationFrame [10] =(uint8_t) encoderDown/3;
		communicationFrame [11] =(uint8_t) PWM/3;





if(counter ==100){
	for (int i =0 ;i<13;i++){

	//	uint8_t *temp = &communicationFrame[i];
			//HAL_UART_Transmit(&huart2,temp, 1, 1000);

	}

		counter=0;


		}
counter++;
   }


	uint32_t setTime = 100;
	uint32_t now,lasttime=0,timechange;
	int error, lasterror,  errorSum=0, Derror;
	int Pout,Iout,Dout;
	int outPWM;


uint8_t calculatePID( int input, int setpoint ,uint8_t P, uint8_t I, uint8_t D ) {



	now = HAL_GetTick ();
	timechange = (now - lasttime);
	if(timechange >=setTime)
	{
		error = setpoint - input;
		errorSum = errorSum+((error +lasterror)/2);
		Derror = error-lasterror;
		Pout = P *error;
		Iout = I*errorSum;
		Dout = D*Derror;

		if (Iout >255){Iout = 255;}
		if(Iout <0){Iout = 0;}

		outPWM = Pout + Iout + Dout;


		if (outPWM >255){outPWM= 255;}
				if(outPWM <0){outPWM = 0;}


				lasterror = error;
				lasttime = now;



	}

	if (input >setpoint)
	{

		kierunek = false;
	}else{


		kierunek = true;
	}

			    return outPWM;
			}



void initWork()
{
	int i=0;
	do{

		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin );
		HAL_Delay(300);
		i++;
	}while(i<10);


}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin );
	//static uint8_t Data[13]; // Tablica przechowujaca wysylana wiadomosc.

	////
	//HAL_UART_Transmit_DMA(&huart1, Data, 40); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
//	HAL_UART_Receive_DMA(&huart2, communicationFrame, 13); // Ponowne włączenie nasłuchiwania



//	frameToName();
//	sprintf(Data, "%s", communicationFrame);
//	HAL_UART_Transmit_DMA(&huart2, Data, 13);

//}





void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart , uint16_t Size)
{
	if(huart->Instance==USART2)
	{

		memcpy( communicationFrame,RxBuf ,Size);

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,RxBuf,RxBuf_SIZE);

	}

}


void send_string(char* s)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}



void firstScreenOled(){

	//  send_string(mesage);


              unsigned char* OledString;
		      OledString =( unsigned char*)sendFrame;
		     // GFX_draw_string(0, 0, OledString, WHITE,BLACK, 1, 1);
		      GFX_draw_string(25, 25, OledString, WHITE,BLACK, 0, 0);
		      GFX_draw_string(0, 45, OledString, WHITE,BLACK, 1, 1);
		      SSD1306_draw_fast_hline(0,43,200,WHITE);
		      GFX_draw_string(0, 33, EncDownMessage , WHITE,BLACK, 1, 1);
		      encoderCounterUp = htim4.Instance->CNT;
		      if (encoderCounterUp >=1000)
		     		      {
		     		    	  itoa(encoderCounterUp,itoaBuffer,10);
		     		    	  		      GFX_draw_string(60, 33, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);


		     		      }else
		     		      {
		     		    	   GFX_draw_string(60, 33, "                  ", WHITE,BLACK, 1, 1);
		     		    	  itoa(encoderCounterUp,itoaBuffer,10);
		     		    	   GFX_draw_string(60, 33, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);


		     		      }


		      SSD1306_draw_fast_hline(0,32,200,WHITE);
		      GFX_draw_string(0, 22, EncUpMessage, WHITE,BLACK, 1, 1);
		      encoderCounterDown = htim3.Instance->CNT;
		      if (encoderCounterDown >=1000)
		  		     		      {
		    	    itoa(encoderCounterDown,itoaBuffer,10);
		    			      GFX_draw_string(60, 22, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);


		  		     		      }else
		  		     		      {
		  		     		    	   GFX_draw_string(60, 22, "                  ", WHITE,BLACK, 1, 1);
		  		     		        itoa(encoderCounterDown,itoaBuffer,10);
		  		     		    		      GFX_draw_string(60, 22, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);


		  		     		      }


		      SSD1306_draw_fast_hline(0,21,200,WHITE);
		      GFX_draw_string(0, 11,PWMMessage , WHITE,BLACK, 1, 1);
		      itoa(PWM,itoaBuffer,10);
		      GFX_draw_string(60, 11, ( unsigned char*)itoaBuffer, WHITE,BLACK, 1, 1);
		      SSD1306_draw_fast_hline(0,10,200,WHITE);
		      GFX_draw_string(0, 0, MODEMessage, WHITE,BLACK, 1, 1);



			  SSD1306_display_repaint();

}




void engineControl()
{





	if (START == 1&& STOP == 0 && manual == 0)
		{


		encoderCounterUp = htim4.Instance->CNT;

        PWM=calculatePID( encoderCounterDown, 1000 , 2,  3,  2 );

		      GFX_draw_string(0, 0,messageStart, WHITE,BLACK, 1, 1);
		      SSD1306_display_repaint();
			//PWM =calculatePID(0,encoderCounterUp);
			 if(kierunek)
						            {


						             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_SET);//podlaczenie sterowania silnikiem
						             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);//podlaczenie sterowania silnikiem
						             HAL_Delay(10);
						             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
						            }else{



						            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
						            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_SET);
						            	HAL_Delay(10);
						            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
						            }
		}
		if (START == 0 && STOP == 1 && manual == 0)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
									            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
									            	HAL_Delay(10);

			 GFX_draw_string(0, 0, stopMessage, WHITE,BLACK, 1, 1);

			         SSD1306_display_repaint();
		}
		if (manual == 1 && leftButton==1 && rightButton == 0)
				{

			 GFX_draw_string(0, 0, messageManualLeft, WHITE,BLACK, 1, 1);
					      SSD1306_display_repaint();

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_SET);
					HAL_Delay(10);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);
				}
		if (manual == 1 && leftButton==0 && rightButton == 1)
						{


			GFX_draw_string(0, 0, messageManualRight, WHITE,BLACK, 1, 1);
								      SSD1306_display_repaint();

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);
					HAL_Delay(10);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);
						}





}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
    MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
 //SSD1306_init();
  SSD1306_init();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,RxBuf,RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);

 // __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
 // HAL_UART_Receive_DMA(&huart2, communicationFrame, 13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // enkoder dol
  	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // enkoder gora
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // wysterowanie silnika
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    initWork();
    firstScreenOled();



  while (1)
  {



	  frameToName();
	  errorCheck();
	  frameTransform();
	  firstScreenOled();


	    engineControl();


	 // PWM=arm_pid_q15(arm_pid,encoderCounterUp);


	   // encoderCounterUp = htim4.Instance->CNT;
		//encoderCounterDown = htim3.Instance->CNT;
		//itoa(encoderCounterUp,itoaBuffer,10);
		//send_string(itoaBuffer);
	      //   	itoa(encoderCounterDown,itoaBuffer,10);
			//	send_string(itoaBuffer);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

