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
#include "NRF24L01.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include "stm32h7xx_hal.h"
//#define ALLOWED_POSITION_OFFSET 5
//#define MAX_UTF16_CHAR_SIZE 2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t adc1,adc2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TRIG_PIN GPIO_PIN_15
//#define TRIG_PORT GPIOE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
int _write(int file, unsigned char * p, int len)
{
HAL_UART_Transmit(&huart3, p, len, 10);
return len;
}
/////////////choumpa/////

/*uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;
*/
enum notes {
  C = 956,
  D = 852,
  E = 758,
  F = 716,
  G = 638
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//void delay (uint16_t time)
//{
//	__HAL_TIM_SET_COUNTER(&htim2, 0);
//	while (__HAL_TIM_GET_COUNTER (&htim2) < time);
//}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RxAddress[] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32];
uint8_t data[50];

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_FilterTypeDef sFilterConfig1;
FDCAN_FilterTypeDef sFilterConfig2;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData_Node3_To_Node1[16];
uint8_t TxData_Node3_To_Node3[16];
uint8_t RxData_From_Node3[16];
uint8_t RxData_From_Node1[16];
uint8_t RxData_From_Node4[8];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
   if(FDCAN1 == hfdcan->Instance)
   {
	  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	  {

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData_From_Node3) != HAL_OK)
		{
		Error_Handler();
		}

	  }
   }

 }//choumpa jodo
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
   if(FDCAN1 == hfdcan->Instance)
   {
	  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	  {
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData_From_Node1) != HAL_OK)
		{
		Error_Handler();
		}

	  }
   }
 }//rider
void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{

    if (FDCAN1 == hfdcan->Instance)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_BUFFER0, &RxHeader, RxData_From_Node4) != HAL_OK)
        {
            Error_Handler();
        }
    }

}//raspi




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	  HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
//	  HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
////jodo///
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//////////////nrf///////////
  NRF24_Init();
  NRF24_RxMode(RxAddress, 10);//nrf rx
  NRF24_ReadAll(data);//nrf rx
////////////////////////////

////////////motor///////////
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//motor back1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//motor back2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//motor front
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14,1);//motor enable1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);//motor enable2
////////////////////////////

///////////jodo///////////
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//led jodo
//////////jodo////////////



//////////////buzzer/////////
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  ////////////////////////
  int a[14];
  int b[2];
//nrf고치기 ,라즈베리 받을때 nrf안받도록 수정 하기 라즈베리 r,l일때 nrf쪽 따로돌게 수정
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  for(int i=0;i<11;i++)
	  {a[i]=RxData_From_Node3[i]-'0';}

	  for(int j=0;j<3;j++)
	  {b[j]=RxData_From_Node1[j]-'0';}

	  int Distance1 = 100* a[1]  +10*a[2] +a[3];
	  int Distance2 = 100* a[4]  +10*a[5] +a[6];
	  int Distance3 = 100* a[8]  +10*a[9] +a[10];
	  int Distance4 = 100* b[0]  +10*b[1] +b[2];//rider


	  printf("    %d %d %d   ",b[0],b[1],b[2]);

///////////////////////////////

///////////////nrf//////////////
	  if (isDataAvailable(2) == 1)
	  	 {
		  NRF24_Receive(RxData);
	  	 }

///////////////////////////////////

//////////////////////buzzer/////////////////
//for (int i=0;i<5;i++){
//	  TIM2->ARR = C;
//	  TIM2->CCR1 = TIM2->ARR/2 ;
//	  HAL_Delay(500);
//	  TIM2->CCR1 = 0;
//	  HAL_Delay(100);
//	  }



	  if(Distance1<=15)
	 	  {
	 	  TIM2->ARR = C;
	 	  TIM2->CCR1 = TIM2->ARR / 2;
	 	  HAL_Delay(50);
	 	  TIM2->CCR1 = 0;
	 	  HAL_Delay(50);
	 	  }

	  if(Distance2<=15)
	 	  {
	 	  TIM2->ARR = C;
	 	  TIM2->CCR1 = TIM2->ARR / 2;
	 	  HAL_Delay(50);
	 	  TIM2->CCR1 = 0;
	 	  HAL_Delay(50);
	 	  }

	  if(Distance3<=15)
	 	  {
	 	  TIM2->ARR = C;
	 	  TIM2->CCR1 = TIM2->ARR / 2;
	 	  HAL_Delay(50);
	 	  TIM2->CCR1 = 0;
	 	  HAL_Delay(50);
	 	  }


 /////////////jodo///////////////

 for(int l=12;l<=14;l++)
 {a[l]=RxData_From_Node3[l]-'0';}
 int jodo = 100* a[12]  +10*a[13] +a[14];
      htim3.Instance->CCR1=jodo;
      if (jodo<45)
      {
    	  htim3.Instance->CCR1=0;
      }

      ////////////////////////////////////

///////////////////////////////////////////////





///////////////go and back (switch)/////////////////

//	  if(RxData[2]==0)
//		  {
//		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,0);
//		  htim1.Instance->CCR1=RxData[0];
//		  htim1.Instance->CCR2=RxData[0];
//		  if(RxData[0]>=100)
//		  	  {
//		  		  htim1.Instance->CCR1=99;
//		  		  htim1.Instance->CCR2=99;
//		  	  }
//		  }
//	  if(RxData[2]==1)
//		  {
//		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,1);
//		  htim1.Instance->CCR1=RxData[0];
//		  htim1.Instance->CCR2=RxData[0];
//		  if(RxData[0]>=100)
//		  	  {
//		  		  htim1.Instance->CCR1=99;
//		  		  htim1.Instance->CCR2=99;
//		  	  }
//		  }

/////////////////////////////////////////////////////

///////////////////////nrf networking and direction//////////////////

	if(RxData[3]==0)
	{


			  if (RxData[1]  <=40)
				  {


					  while (RxData[1]  <= 40)
					  {

						  if (isDataAvailable(2) == 1)
						  {
							  NRF24_Receive(RxData);
						  }

						  htim1.Instance->CCR3 = 80;
						  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
						  HAL_Delay(100);
						  if(RxData[2]==0)
							  {
							  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
							  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,0);
							  htim1.Instance->CCR1=RxData[0];
							  htim1.Instance->CCR2=RxData[0];
							  if(RxData[0]>=100)
								  {
									  htim1.Instance->CCR1=99;
									  htim1.Instance->CCR2=99;
								  }
							  }
						  if(RxData[2]==1)
							  {
							  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
							  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,1);
							  htim1.Instance->CCR1=RxData[0];
							  htim1.Instance->CCR2=RxData[0];
							  if(RxData[0]>=100)
								  {
									  htim1.Instance->CCR1=99;
									  htim1.Instance->CCR2=99;
								  }
							  }

						  if(41<=RxData[1]&&RxData[1]<=80)
						  {
							  htim1.Instance->CCR3 = 90;
							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
							  HAL_Delay(200);
							  htim1.Instance->CCR3 = 0;
						  }


					  }


				  }

			  if (RxData[1] > 85)
					  {

						  while (RxData[1] > 85)
						  {

							  if (isDataAvailable(2) == 1)
							  {
								  NRF24_Receive(RxData);
							  }
							  htim1.Instance->CCR3 = 80;
							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
							  HAL_Delay(100);
							  if(RxData[2]==0)
								  {
								  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
								  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,0);
								  htim1.Instance->CCR1=RxData[0];
								  htim1.Instance->CCR2=RxData[0];
								  if(RxData[0]>=100)
									  {
										  htim1.Instance->CCR1=99;
										  htim1.Instance->CCR2=99;
									  }
								  }
							  if(RxData[2]==1)
								  {
								  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
								  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,1);
								  htim1.Instance->CCR1=RxData[0];
								  htim1.Instance->CCR2=RxData[0];
								  if(RxData[0]>=100)
									  {
										  htim1.Instance->CCR1=99;
										  htim1.Instance->CCR2=99;
									  }
								  }

							  if(41<=RxData[1]&&RxData[1]<=85)
							  {
								  htim1.Instance->CCR3 = 90;
								  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
								  HAL_Delay(200);
								  htim1.Instance->CCR3 = 0;
							  }
						  }
					  }



	  }
////////////////////////////////////////////////////////////

///////////////go and back /////////////////

						if(RxData[2]==0)
							  {
							  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
							  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,0);
							  htim1.Instance->CCR1=RxData[0];
							  htim1.Instance->CCR2=RxData[0];
							  if(RxData[0]>=100)
								  {
									  htim1.Instance->CCR1=99;
									  htim1.Instance->CCR2=99;
								  }
							  //////////////ridar///////////////
							  	  if(Distance4<=20)
							  	  {
							  		  htim1.Instance->CCR1=0;
							  		  htim1.Instance->CCR2=0;
							  	  }

							  }
						  if(RxData[2]==1)
							  {
							  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
							  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,1);
							  htim1.Instance->CCR1=RxData[0];
							  htim1.Instance->CCR2=RxData[0];
							  if(RxData[0]>=100)
								  {
									  htim1.Instance->CCR1=99;
									  htim1.Instance->CCR2=99;
								  }
							  //////////////ridar///////////////
							  	  if(Distance4<=20)
							  	  {
							  		  htim1.Instance->CCR1=0;
							  		  htim1.Instance->CCR2=0;
							  	  }

							  }


////////////////////////////////////////////////////

/////////////////can fd raspi and direction ////////////////////
		  	  if(RxData[3]==1)
		  	  {


		  		  if (RxData_From_Node4[0]=='L')
		  				  {
		  					  while (RxData_From_Node4[0]=='L')
		  					  {

		  						  // left motor
		  						  htim1.Instance->CCR3 = 90;
		  						  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
		  						  HAL_Delay(100);

		  						  // straight
		  						  if(RxData_From_Node4[0]=='G')
		  						  {
		  							  htim1.Instance->CCR3 = 90;
		  							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
		  							  HAL_Delay(200);
		  							  htim1.Instance->CCR3 = 0;
		  						  }
		  						  if (40<=RxData[1]&&RxData[1]<=80)
		  						  {
		  							  break;
		  						  }


		  					  }

		  				  }

		  			  if (RxData_From_Node4[0]=='R')
		  					  {



		  						  while (RxData_From_Node4[0]=='R')
		  						  {

		  							  // right motor
		  							  htim1.Instance->CCR3 = 90;
		  							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
		  							  HAL_Delay(100);

		  							  if(RxData_From_Node4[0]=='G')
		  							  {
		  								  htim1.Instance->CCR3 = 90;
		  								  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
		  								  HAL_Delay(200);
		  								  htim1.Instance->CCR3 = 0;
		  							  }
		  							  if (40<=RxData[1]&&RxData[1]<=80)
		  							  {
		  								  break;
		  							  }


		  						  }


		  					  }

		  	  }
///////////////////////////////////////////////////////////////////

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 32;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 5;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 3;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_16;
  hfdcan1.Init.RxFifo1ElmtsNbr = 1;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_16;
  hfdcan1.Init.RxBuffersNbr = 1;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_16;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_16;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  	  	sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = 1;
        sFilterConfig.RxBufferIndex = 1;
        sFilterConfig.FilterType = FDCAN_FILTER_DUAL; // Ignore because FDCAN_FILTER_TO_RXBUFFE
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = 0x33; // ID Node2
        sFilterConfig.FilterID2 = 0x7ff; // Ignore because FDCAN_FILTER_TO_RXBUFFER
        if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
               {
                  Error_Handler();
               }
//
  	  	sFilterConfig1.IdType = FDCAN_STANDARD_ID;
        sFilterConfig1.FilterIndex = 2;
        sFilterConfig1.RxBufferIndex = 2;
        sFilterConfig1.FilterType = FDCAN_FILTER_DUAL; // Ignore because FDCAN_FILTER_TO_RXBUFFE
        sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        sFilterConfig1.FilterID1 = 0x11; // ID Node2
        sFilterConfig1.FilterID2 = 0x7ff; // Ignore because FDCAN_FILTER_TO_RXBUFFER
//
        if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1) != HAL_OK)
        {
           Error_Handler();
        }
  	  	sFilterConfig2.IdType = FDCAN_STANDARD_ID;
        sFilterConfig2.FilterIndex = 0;
        sFilterConfig2.RxBufferIndex = 0;
        sFilterConfig2.FilterType = FDCAN_FILTER_DUAL; // Ignore because FDCAN_FILTER_TO_RXBUFFE
        sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
        sFilterConfig2.FilterID1 = 0x44; // ID Node2
        sFilterConfig2.FilterID2 = 0x7ff; // Ignore because FDCAN_FILTER_TO_RXBUFFER

        if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig2) != HAL_OK)
               {
                  Error_Handler();
               }

        if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
              {
                Error_Handler();
              }





        if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0) != HAL_OK)
          {
            /* Notification Error */
            Error_Handler();
          }
            if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
              {
                Error_Handler();
              }
            if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
              {
                Error_Handler();
              }

            TxHeader.Identifier = 0x22;
            TxHeader.IdType = FDCAN_STANDARD_ID;
            TxHeader.TxFrameType = FDCAN_DATA_FRAME;
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
            TxHeader.FDFormat = FDCAN_FD_CAN;
            TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            TxHeader.MessageMarker = 0x0;




  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 65;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 128-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 64;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ce_Pin|csn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, IRQ_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ce_Pin csn_Pin */
  GPIO_InitStruct.Pin = ce_Pin|csn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : IRQ_Pin PG14 */
  GPIO_InitStruct.Pin = IRQ_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

