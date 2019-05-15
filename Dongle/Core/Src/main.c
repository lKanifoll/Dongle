/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
//#include "ring_buffer.h"
#include "modbus.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//------------------------------------------------------------------ for rx from mcu
typedef struct 
{
	uint8_t		prefix;
	uint8_t		byte_count;
	uint8_t		function;     
	uint8_t		data_buff[BUFFER_SIZE]; 
} rx_frame_t; 

typedef union 
{
	rx_frame_t  rx_frame;
	uint8_t		rx_raw_frame[BUFFER_SIZE];
} rx_union_u; 

rx_union_u rx_raw;

uint8_t rx_size;
uint8_t odd_flag;
//------------------------------------------------------------------

//------------------------------------------------------------------ for tx to mcu
typedef struct 
{
	uint8_t		prefix;
	uint8_t		byte_count;
	uint8_t		function;     
	uint8_t		data_buff[BUFFER_SIZE]; 
} tx_frame_t; 

typedef union 
{
	tx_frame_t  tx_frame;
	uint8_t		tx_raw_frame[BUFFER_SIZE];
} tx_union_u; 

tx_union_u tx_raw;

//uint8_t rx_size;
//uint8_t odd_flag;
//------------------------------------------------------------------

//uint8_t tmp_buff[BUFFER_SIZE] ;
//uint16_t rx_buff[BUFFER_SIZE];
//uint8_t	tx_buff[BUFFER_SIZE];
//uint8_t modbus_slave_cmd[11] = { 0xE4, 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x49, 0xAD };
//uint8_t tmp_rmp[1] = { 0xAA };

//uint8_t modbus_master_cmd[BUFFER_SIZE];

//ring_buff_t ring_buffer;
//uint16_t start_ptr;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef	htim14;
UART_HandleTypeDef	huart1;
UART_HandleTypeDef	huart2;
DMA_HandleTypeDef	hdma_usart1_rx;
DMA_HandleTypeDef	hdma_usart1_tx;
DMA_HandleTypeDef	hdma_usart2_rx;
DMA_HandleTypeDef	hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
void modbus_handler();
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
	//HAL_UART_Receive_DMA(&huart1, rx_485_buff, sizeof(rx_485_buff));
  HAL_UART_Receive_DMA(&huart1, modbus_raw.modbus_master_frame, sizeof(modbus_raw.modbus_master_frame));
	//HAL_UART_Receive_DMA(&huart2, tmp_buff, 10);
  HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  if (modbus_rx_complete)
	  {
		  modbus_rx_complete = FALSE;
		  modbus_handler();
	  }
  }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN 4 */
void modbus_handler()
{
	modbus_tx_complete = TRUE;
	bzero(rx_raw.rx_raw_frame, BUFFER_SIZE);
	bzero(tx_raw.tx_raw_frame, BUFFER_SIZE);
	switch (modbus_raw.modbus_frame.function)
	{
	case MODBUS_READ_HOLDING_REG:
		
		switch (__builtin_bswap16(modbus_raw.modbus_frame.start_reg_address))
		{
		case reg_UUID:
			
			HAL_UART_Transmit_IT(&huart2, read_UUID, sizeof(read_UUID));			
			rx_size = 10;
			break;
		case reg_UDID:
			HAL_UART_Transmit_IT(&huart2, read_UDID, sizeof(read_UDID));
			rx_size = 20;
			break;
		case reg_SETTINGS:
			HAL_UART_Transmit_IT(&huart2, read_SETTINGS, sizeof(read_SETTINGS));
			rx_size = 15;
			odd_flag = TRUE;
			break;
		case reg_DATE:
			HAL_UART_Transmit_IT(&huart2, read_DATE, sizeof(read_DATE));
			rx_size = 19;
			odd_flag = TRUE;
			break;
		case reg_WEEK_PTS:
			HAL_UART_Transmit_IT(&huart2, read_WEEK_PTS, sizeof(read_WEEK_PTS));
			rx_size = 11;
			odd_flag = TRUE;
			break;
		case reg_CUSTOM_DAY_PTS:
			HAL_UART_Transmit_IT(&huart2, read_CUSTOM_DAY_PTS, sizeof(read_CUSTOM_DAY_PTS));
			rx_size = 28;
			break;
		default:
			//return illigal data address
			break;
		}
		break;
		
	case MODBUS_WRITE_MULTIPLY_REG:
		
		tx_raw.tx_frame.prefix = 0xAA;											// prefix is similar for all frames
		
		
		switch (__builtin_bswap16(modbus_raw.modbus_frame.start_reg_address))
		{
		case reg_UDID:
			tx_raw.tx_frame.byte_count = modbus_raw.modbus_frame.data_buff[0] + 1; 	// + 1 byte cmd
			tx_raw.tx_frame.function = 0x06;
			memcpy(tx_raw.tx_frame.data_buff, (modbus_raw.modbus_frame.data_buff + 1), modbus_raw.modbus_frame.data_buff[0]);
			tx_raw.tx_frame.data_buff[modbus_raw.modbus_frame.data_buff[0]] = checksum8(tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 2);
			
			HAL_UART_Transmit_IT(&huart2, tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 3); // size with checksum8
			rx_size = 20;
			break;
		case reg_SETTINGS:
			tx_raw.tx_frame.byte_count = modbus_raw.modbus_frame.data_buff[0]; 	// byte count is 12 but significant bytes is 11, + 1 byte cmd
			tx_raw.tx_frame.function = 0x0A;
			memcpy(tx_raw.tx_frame.data_buff, (modbus_raw.modbus_frame.data_buff + 1), modbus_raw.modbus_frame.data_buff[0] - 1); // - 1 non significant byte
			tx_raw.tx_frame.data_buff[modbus_raw.modbus_frame.data_buff[0] - 1] = checksum8(tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 2);
			
			HAL_UART_Transmit_IT(&huart2, tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 3);  // size with checksum8
			rx_size = 15;
			odd_flag = TRUE;
			break;
		case reg_DATE:
			tx_raw.tx_frame.byte_count = modbus_raw.modbus_frame.data_buff[0] + 8;  	// byte count for frame to set Date is 8 byte 0x00 and 7 byte Date (7 byte + 1 non signif.), + 1 byte cmd
			tx_raw.tx_frame.function = 0xA7;
			memcpy((tx_raw.tx_frame.data_buff + 8), (modbus_raw.modbus_frame.data_buff + 1), modbus_raw.modbus_frame.data_buff[0] - 1);  // "+ 8" - offset for 8 byte 0x00, "- 1" - non significant byte
			tx_raw.tx_frame.data_buff[modbus_raw.modbus_frame.data_buff[0] + 7] = checksum8(tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 2);
			
			HAL_UART_Transmit_IT(&huart2, tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 3);   // size with checksum8
			rx_size = 19;
			odd_flag = TRUE;
			break;
		case reg_WEEK_PTS:
			tx_raw.tx_frame.byte_count = modbus_raw.modbus_frame.data_buff[0];  	// byte count is 12 but significant bytes is 11, + 1 byte cmd
			tx_raw.tx_frame.function = 0xA8;
			memcpy(tx_raw.tx_frame.data_buff, (modbus_raw.modbus_frame.data_buff + 1), tx_raw.tx_frame.byte_count - 1);   // - 1 non significant byte
			tx_raw.tx_frame.data_buff[tx_raw.tx_frame.byte_count - 1] = checksum8(tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 2);
			
			HAL_UART_Transmit_IT(&huart2, tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 3);   // size with checksum8
			
			rx_size = 11;
			odd_flag = TRUE;
			break;
		case reg_CUSTOM_DAY_PTS:
			tx_raw.tx_frame.byte_count = modbus_raw.modbus_frame.data_buff[0] + 1;  	// + 1 byte cmd
			tx_raw.tx_frame.function = 0xA9;
			memcpy(tx_raw.tx_frame.data_buff, (modbus_raw.modbus_frame.data_buff + 1), modbus_raw.modbus_frame.data_buff[0]);
			tx_raw.tx_frame.data_buff[modbus_raw.modbus_frame.data_buff[0]] = checksum8(tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 2);
			
			HAL_UART_Transmit_IT(&huart2, tx_raw.tx_raw_frame, tx_raw.tx_frame.byte_count + 3);  // size with checksum8
			rx_size = 28;
			break;
		default:
			//return illigal data address
			break;
		}
		
		break;
	default:
		//return ILLEGAL FUNCTION
		break;
	}
	HAL_UART_Receive_IT(&huart2, rx_raw.rx_raw_frame, rx_size);
	
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{

	}
	else 
	if (huart == &huart2)
	{
		if (checksum8(rx_raw.rx_raw_frame, (rx_size - 1)) ==  rx_raw.rx_raw_frame[rx_size - 1])
		{
			//prepare modbus_slave_frame
			switch(modbus_raw.modbus_frame.function)
			{
			case MODBUS_READ_HOLDING_REG:
				
				bzero(modbus_03_raw.modbus_frame.data_buff, BUFFER_SIZE);
				modbus_03_raw.modbus_frame.address = modbus_raw.modbus_frame.address;
				modbus_03_raw.modbus_frame.function = modbus_raw.modbus_frame.function;
				if (odd_flag) 
				{
					if ((__builtin_bswap16(modbus_raw.modbus_frame.start_reg_address)) == reg_DATE)
					{
						modbus_03_raw.modbus_frame.byte_count = rx_raw.rx_frame.byte_count - 8;
						memcpy(modbus_03_raw.modbus_frame.data_buff, rx_raw.rx_frame.data_buff + 8, modbus_03_raw.modbus_frame.byte_count - 1);
					}
					else
					{
						modbus_03_raw.modbus_frame.byte_count = rx_raw.rx_frame.byte_count;
						memcpy(modbus_03_raw.modbus_frame.data_buff, rx_raw.rx_frame.data_buff, modbus_03_raw.modbus_frame.byte_count - 1);
					}
				}
				else
				{
					modbus_03_raw.modbus_frame.byte_count = rx_raw.rx_frame.byte_count - 1;   						// byte_count in mcu protocol count with command byte, that's why (byte_count - 1)
					memcpy(modbus_03_raw.modbus_frame.data_buff, rx_raw.rx_frame.data_buff, modbus_03_raw.modbus_frame.byte_count);
				}
				odd_flag = FALSE;
				
				mb_CRC = modbus_rtu_calc_crc(modbus_03_raw.modbus_slave_frame, modbus_03_raw.modbus_frame.byte_count + 3);
				modbus_03_raw.modbus_slave_frame[modbus_03_raw.modbus_frame.byte_count + 3] = mb_CRC;
				modbus_03_raw.modbus_slave_frame[modbus_03_raw.modbus_frame.byte_count + 4] = mb_CRC >> 8;
				
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit_DMA(&huart1, modbus_03_raw.modbus_slave_frame, modbus_03_raw.modbus_frame.byte_count + 5);
				break;	
				
			case MODBUS_WRITE_MULTIPLY_REG:
				bzero(modbus_10_raw.modbus_slave_frame, 8);
				modbus_10_raw.modbus_frame.address = modbus_raw.modbus_frame.address;
				modbus_10_raw.modbus_frame.function = modbus_raw.modbus_frame.function;
				modbus_10_raw.modbus_frame.start_reg_address = modbus_raw.modbus_frame.start_reg_address;
				modbus_10_raw.modbus_frame.reg_count = modbus_raw.modbus_frame.reg_count;				
				modbus_10_raw.modbus_frame.mb_CRC = modbus_rtu_calc_crc(modbus_10_raw.modbus_slave_frame, 6);
				odd_flag = FALSE;
				HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit_DMA(&huart1, modbus_10_raw.modbus_slave_frame, 8);
				break;
			}
		}

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Receive_IT(&huart2, tmp_buff, sizeof(tmp_buff));
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);

	//HAL_UART_Receive_DMA(&huart1, rx_485_buff, sizeof(rx_485_buff));
	//HAL_UART_DMAStop(&huart2);
	//tmp_size = 10;
	//tmp_buff = (uint8_t*)malloc(tmp_size);
	//HAL_UART_Receive_DMA(&huart2, tmp_buff, tmp_size);
}


/* USER CODE END 4 */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  SET_BIT(huart1.Instance->CR1, USART_CR1_IDLEIE);
  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  //SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin|GREEN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DE_Pin */
  GPIO_InitStruct.Pin = DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DE_GPIO_Port, &GPIO_InitStruct);

}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
