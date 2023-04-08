/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//uint8_t RxData_From_Node1[2]={'A','B'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#ifdef __GNUC__
//  /* With GCC, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//  return ch;
//}
int _write(int file, unsigned char * p, int len)
{
HAL_UART_Transmit(&huart1, p, len, 10);
return len;
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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_ADC_Start(&hadc1);

//  uint8_t dir = 0;
//  uint8_t dir = 0;				// 방향?�� ?��???��?�� �????????????????��
//  uint16_t interval = 50;			// Pulse
//  int16_t temp_Interval;			// 계산?�� ?�� ?��?��?��?�� Interval �????????????????��
//  int32_t temp_CCR = 0;			// Pulse �??????????????? 계산 ?�� ?��?�� (TIM5->CCR ?���???????????????)
//  uint32_t turningPoint = 2000;
//  __IO uint32_t CCR1;
  htim3.Instance->CCR1=0;
  htim3.Instance->CCR2=0;
  uint32_t adc_value;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	      adc_value = HAL_ADC_GetValue(&hadc1);
	      printf("adc_value = %d\r\n", (int)adc_value);
	    }
	  HAL_Delay(100);
//
//		  htim3.Instance->CCR1=(100*(adc_value/4095));
//		  HAL_Delay(10);
//		  htim3.Instance->CCR2=(100*(adc_value/4095));
//		  HAL_Delay(10);
	  if((adc_value>4000)&&(adc_value<=4095))
	  {
		  htim3.Instance->CCR1=0;
		  htim3.Instance->CCR2=0;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
		  HAL_Delay(10);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
			 HAL_Delay(10);
	  }
	  if((adc_value>3000)&&(adc_value<4000))
	  {
		  htim3.Instance->CCR1=30;
		  htim3.Instance->CCR2=30;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
		  HAL_Delay(10);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
			 HAL_Delay(10);
	  }

	  if((adc_value>2000)&&(adc_value<3000))
	  {
		  htim3.Instance->CCR1=50;
		  htim3.Instance->CCR2=50;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
		  HAL_Delay(10);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
			 HAL_Delay(10);
	  }
	  if((adc_value>1000)&&(adc_value<2000))
	 	  {
	 		  htim3.Instance->CCR1=80;
	 		  htim3.Instance->CCR2=80;
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
			  HAL_Delay(10);
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
				 HAL_Delay(10);
	 	  }
	  if((adc_value>0)&&(adc_value<1000))
	  	 	  {
	  	 		  htim3.Instance->CCR1=100;
	  	 		  htim3.Instance->CCR2=100;
	  			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	  			  HAL_Delay(10);
	  				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
	  				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
	  				 HAL_Delay(10);
	  	 	  }
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
//	  HAL_Delay(10);
//	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
//	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
//	 HAL_Delay(10);






//	  if((htim3.Instance->CCR1)==70)
//	  {
//		  htim3.Instance->CCR1--;
//		  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
//		  	  HAL_Delay(500);
//	  }

//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);
//	  HAL_Delay(500);
//	  if(temp_CCR >= (TIM3->ARR + 1))				// 최고 ?��?�� ?��?�� 찍었?�� ?��
//	  	  {
//	  		temp_CCR = TIM3->ARR + 1;					// 10000
//	  		temp_Interval = -interval;					// �????????????????��?�� 감소
//	  	  }
//	  	  else if(temp_CCR <= turningPoint)				// 최�? ?��?�� ?��?�� 찍었?�� ?��
//	  	  {
//	  		temp_CCR = turningPoint;
//	  		temp_Interval = interval;					// �????????????????��?�� 증�?
//	  		dir = !dir;									// 방향 �???????????????�???????????????
//	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, dir);	// PG10 ?���??????????????? ?��?��
//	  	  }
//	  	  temp_CCR += temp_Interval;					// ?��?��?�� CCR �??????????????? 계산
//	  	  if(temp_CCR == 0)
//	  	  {
//	  		  TIM3->CCR1 = temp_CCR;					// Underflow 방�??��
//	  	  }
//	  	  else
//	  	  {
//	  		  TIM3->CCR1 = temp_CCR - 1;				// 기본 계산 (256?? 0~255 ?��???��?�� -1 ?��?��?��)
//	  	  }
//


		//dir = !dir;									// 방향 �????????????????�????????????????
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, dir);
//		HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 19;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC2 PC3 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

