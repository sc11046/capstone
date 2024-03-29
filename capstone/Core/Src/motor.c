/*
 * motor.c
 *
 *  Created on: 2023. 8. 7.
 *      Author: sc110
 */

#include "motor.h"
#include "main.h"


enum notes {
  C = 956,
  D = 852,
  E = 758,
  F = 716,
  G = 638
};
void go_back(void){
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
			  light_sensor();
			  ridar();
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
				light_sensor();
				ridar();
			  }
}

void nrf_motor (void){
	if(RxData[3]==0)
	{
			  if (RxData[1]  <=70)
				  {
					  while (RxData[1]  <= 70)
					  {

						  light_sensor();
						       buzzer();
						  if (isDataAvailable(2) == 1)
						  {
							  NRF24_Receive(RxData);
						  }
						  htim1.Instance->CCR3 = 80;
						  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
						  HAL_Delay(100);
						  go_back();

						  if(70<=RxData[1]&&RxData[1]<=110)
						  {
							  htim1.Instance->CCR3 = 90;
							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
							  HAL_Delay(200);
							  htim1.Instance->CCR3 = 0;
						  }
					  }
				  }
			  if (RxData[1] > 110)
					  {
						  while (RxData[1] > 110)
						  {
							  light_sensor();
							       buzzer();
							  if (isDataAvailable(2) == 1)
							  {
								  NRF24_Receive(RxData);
							  }
							  htim1.Instance->CCR3 = 80;
							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
							  HAL_Delay(100);
							  go_back();
							  if(70<=RxData[1]&&RxData[1]<=110)
							  {
								  htim1.Instance->CCR3 = 90;
								  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
								  HAL_Delay(200);
								  htim1.Instance->CCR3 = 0;
							  }

						  }
					  }


	  }
}

void rpi_motor (void){
	  if(RxData[3]==1)
	  {
		  if (RxData_From_Node4[0]=='L')
				  {
					  while (RxData_From_Node4[0]=='L')
					  {
						  light_sensor();
						       buzzer();
						  if (isDataAvailable(2) == 1)
						  {
							  NRF24_Receive(RxData);
						  }
						  htim1.Instance->CCR3 = 80;
						  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
						  HAL_Delay(100);
						  go_back();

						  if(RxData_From_Node4[0]=='G')
						  {
							  htim1.Instance->CCR3 = 90;
							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
							  HAL_Delay(200);
							  htim1.Instance->CCR3 = 0;
							break;
						  }

					  }
				  }

			  if (RxData_From_Node4[0]=='R')
					  {
						  while (RxData_From_Node4[0]=='R')
						  {
								   light_sensor();
							       buzzer();
							  if (isDataAvailable(2) == 1)
							  {
								  NRF24_Receive(RxData);
							  }
							  htim1.Instance->CCR3 = 80;
							  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
							  HAL_Delay(100);
							  go_back();
							  if(RxData_From_Node4[0]=='G')
							  {
								  htim1.Instance->CCR3 = 90;
								  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
								  HAL_Delay(200);
								  htim1.Instance->CCR3 = 0;
								break;
							  }


						  }
					  }
	  }
}
