/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> //s
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };

uint8_t		ButtonArray[2]  = {0};
uint32_t	LEDTimeStamp    = 0;
uint16_t	LEDFrequency    = 0;
uint16_t	LEDHalfPeriod	= 0;
uint8_t		LED_On_Off		= 1;  //On

typedef enum
{
	Menu_Print,
	Wait_Select_Menu,
	Menu_0_Print = 10,
	Menu_0_Select,
	Menu_1_Print =20,
	Menu_1_Select
}State_Menu;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTRecieveAndResponsePolling();
int16_t UARTRecieveIT();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  {
  char temp[]="\n\n\n\r!!!  START  !!!\n\n\n\r";
  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		ButtonArray[0] = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

		/*Method 1 Polling Mode*/

//		UARTRecieveAndResponsePolling();

		/*Method 2 Interrupt Mode*/
		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

		/*Method 2 W/ 1 Char Received*/
		if(LED_On_Off){
					if(HAL_GetTick() - LEDTimeStamp >= LEDHalfPeriod) //ms
						{
							LEDTimeStamp = HAL_GetTick();
							HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
				}
				}
				else {
					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
				}


		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
		{
			sprintf(TxDataBuffer, "\n\rReceivedChar:[%c]\r\n", inputchar);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		}
		static State_Menu  State = Menu_Print;
				switch(State){
					case Menu_Print:
						sprintf(TxDataBuffer, "\n------- Menu -------\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						sprintf(TxDataBuffer, "[0] :LED Control\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						sprintf(TxDataBuffer, "[1] :Button Status\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						State = Wait_Select_Menu;
						break;
					case Wait_Select_Menu:
						switch(inputchar){
							case -1:
								// on input
								break;
							case '0':
								State = Menu_0_Print;
								break;
							case '1':
								State = Menu_1_Print;
								break;
							default:
									sprintf(TxDataBuffer, "\n\n\rOnly [0],[1] that can be pressable.\n\n\r");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
									State = Menu_Print;
									break;
						}
						break;
//-----------------------------------------------------------------------------------------------------------------------------
					case Menu_0_Print:
						sprintf(TxDataBuffer, "\n------- Menu[0] -------\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						sprintf(TxDataBuffer, "[a] :Speed Up 	+1 Hz.\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						sprintf(TxDataBuffer, "[s] :Speed Down -1 Hz.\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						sprintf(TxDataBuffer, "[d] :On/Off\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						sprintf(TxDataBuffer, "[x] :Back\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						State = Menu_0_Select;
						break;
					case Menu_0_Select:
						switch (inputchar) {
							case -1:
								//on input
								break;
							case 'a':
								LEDFrequency +=1;
								LEDHalfPeriod = 500/LEDFrequency;
								sprintf(TxDataBuffer, "\n\rLED_ON at %d Hz\n\n\r", LEDFrequency);
								HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
								State = Menu_0_Print;
								break;
							case 's':
								if(LEDFrequency!=0){
								LEDFrequency -=1;
								LEDHalfPeriod = 500/LEDFrequency;
								sprintf(TxDataBuffer, "\n\rLED_ON at %d Hz\n\n\r", LEDFrequency);
								HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
								State = Menu_0_Print;
								}
								else {
									sprintf(TxDataBuffer, "Frequency can't be Negative\n\r", LEDFrequency);
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
								}
								State = Menu_0_Print;
								break;
							case 'd':
								if (LED_On_Off == 1){
									sprintf(TxDataBuffer, "\n\rLED ON\n\n\r");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
									}
								else {
									sprintf(TxDataBuffer, "\n\rLED OFF\n\n\r");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
									}
								LED_On_Off = !LED_On_Off;
								State = Menu_0_Print;
								break;
							case 'x':
								State = Menu_Print;
								break;
							default:
								sprintf(TxDataBuffer, "\n\rOnly [a],[s],[d],[x] that can be pressable.\n\n\r");
								HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
								State = Menu_0_Print;
								break;
						}
						break;
//-----------------------------------------------------------------------------------------------------------------------------
					case Menu_1_Print:
						sprintf(TxDataBuffer, "\n-------Button Status-------\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						sprintf(TxDataBuffer, "[x] :Back\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						State = Menu_1_Select;
						break;
					case Menu_1_Select:
						switch (inputchar) {
							case -1:
								//on input
								break;
							case 'x':
								State = Menu_Print;
								break;
							default:
								sprintf(TxDataBuffer, "\n\n\rOnly [x],[Button Blue] that can be pressable.\n\r");
								HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
								State = Menu_1_Print;
								break;
									}
						if (ButtonArray[0]== 0 && ButtonArray[1]== 1){
							sprintf(TxDataBuffer, "\n\n\n\rButton PRESSED\n\n\r");
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
							State = Menu_1_Print;
						}
						else if (ButtonArray[0] == 1 && ButtonArray[1] == 0) {
							sprintf(TxDataBuffer, "\n\n\n\rButton UNPRESSED\n\n\r");
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
							State = Menu_1_Print;
						}
						break;

}
				ButtonArray[1] = ButtonArray[0];


		/*This section just simmulate Work Load*/
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UARTRecieveAndResponsePolling()
{
	char Recieve[32]={0};

	HAL_UART_Receive(&huart2, (uint8_t*)Recieve, 32, 1000);

	sprintf(TxDataBuffer, "Received:[%s]\r\n", Recieve);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);

}


int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
