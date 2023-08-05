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
#include "displayAndControl2.h"
#include "engineControl2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RxBuf_SIZE 35


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

  // uint8_t communicationFrame  [13] = BIT STARTU= 0xFF , P , I , D , START , STOP , buttonLeft , buttonRight  , manual ,encoderUp,encoderDown,PWM, BIT STOPU = 0x00
 UART_HandleTypeDef huart2;
 DMA_HandleTypeDef hdma_usart2_rx;
 // P,I,D,Start,STOP,LeftButton,RighrButton,manual,encoderUp,encoderDown,PWM
 uint8_t  communicationFrame [RxBuf_SIZE]; // table with data from PC
 uint8_t  RxBuf[RxBuf_SIZE]; // buffer
 uint32_t control;
 uint32_t torque=0; // value of calculated engine torque
 uint8_t static sendCounter=0; // counter witch is controling sending information
 int controlOfComm ; // variable witch is controling flow of application


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


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart , uint16_t Size)
{
	if(huart->Instance==USART2)
	{

		for (int i = 0 ; i<RxBuf_SIZE;i++){
						memcpy( &communicationFrame[i],&RxBuf[i] ,Size);

				}



		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,RxBuf,RxBuf_SIZE);

		controlOfComm =1;// one if communication arrived

	}


}


void send_string(char* s)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
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
  // DMA before usart !!
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
  // DMA before usart !!

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,RxBuf,RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // engine power control
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // engine direction control
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // engine direction control
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // enkoder down
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // encoder up
    SSD1306_init(); // initialization of oled
    firstScreenOled();// painted first oled
    initWork(); // information of ended initialization


  while (1)
  {



	  oledFirstStageControl(communicationFrame); // paint first stage
	  encoderDownstageControl(); // painting encoder down value
	  encoderUpstageControl(); // paiting encoder up value
	  engineControl(communicationFrame, &torque, &controlOfComm); // control engine and calculate pid values

		 if (sendCounter >250)
					            {
			 sending(&torque);// sending information to PC
  	         sendCounter=sendCounter-250;
             }else {

		     sendCounter++;

		            }





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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

