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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280.h"

/* REGISTER ADDRESS REGISTER NAME    READ/WRITE 	DESCRIPTION 					POR STATE
  0x12H 				ChargeOption()   Read or Write  Charger Options Control 		0xF902H
  0x14H 				ChargeCurrent()  Read or Write  7-Bit Charge Current Setting 	0x0000H
  0x15H 				ChargeVoltage()  Read or Write  11-Bit Charge Voltage Setting 	0x0000H
  0x3FH 				InputCurrent()   Read or Write  6-Bit Input Current Setting 	0x1000H
  0XFEH 				ManufacturerID() Read Only 		Manufacturer ID 				0x0040H
  0xFFH 				DeviceID()       Read Only 		Device ID 						0x000BH
  
*/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define time_uart 0.002084
#define ID_STM32 0x01
// 2.04 ms
int value = 55;
float tf = 0.0f, pf = 0.0f, af = 0.0f, hf = 0.0f;
uint16_t temp = 0;
uint8_t addr = 0;
uint8_t value1[2] = {0, 0};
uint8_t str;
uint8_t cnt = 0;
uint8_t buff_uart[255];
uint8_t delayByte = 0;
uint8_t bufferrx[2] = {0, 0};
uint8_t buffertx[1] = {0xff};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SMBUS_HandleTypeDef hsmbus1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_SMBUS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t GenCRC16(uint8_t* buff, size_t len);
uint8_t CheckCRC16(uint8_t* buff, size_t len);
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_I2C1_SMBUS_Init();
  /* USER CODE BEGIN 2 */
  
  // Выключение RGB диодов
  //HAL_GPIO_WritePin(GPIOB, LED_B_Pin|LED_G_Pin|LED_R_Pin, GPIO_PIN_SET);
  
  // Активация всех модулей питания 12V, 5V, 3.3V, 6V.
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, EN_12LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, EN_5V_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, EN_3_3V_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, EN_6V_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, EN_RS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  
  // Инициализация датчика BME280
  BME280_Init();
  
  // Включение питания BQ
  HAL_GPIO_WritePin(GPIOA, EN_BQ_Pin, GPIO_PIN_SET);
  
  //Включение приема по modBus
  HAL_GPIO_WritePin(GPIOB, EN_RS_Pin, GPIO_PIN_RESET);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF); // очищаем флаг
  HAL_UART_Receive_IT(&huart1, &str, 1);
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {  
      // получение температуры
      tf = BME280_ReadTemperature();
      temp = tf;   
      
      
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_SMBUS_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hsmbus1.Instance = I2C1;
  hsmbus1.Init.Timing = 0x101098F3;
  hsmbus1.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus1.Init.OwnAddress1 = 2;
  hsmbus1.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus1.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus1.Init.OwnAddress2 = 0;
  hsmbus1.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus1.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus1.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus1.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus1.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
  hsmbus1.Init.SMBusTimeout = 0x000080C3;
  if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 19200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_BQ_Pin|EN_12LED_Pin|EN_3_3V_Pin|EN_6V_Pin
                          |UART_DE_Pin|Light_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_RS_Pin|EN_Hall_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_5V_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_water_sens_GPIO_Port, EN_water_sens_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : EN_BQ_Pin EN_12LED_Pin EN_3_3V_Pin EN_6V_Pin
                           UART_DE_Pin EN_water_sens_Pin Light_LED_Pin */
  GPIO_InitStruct.Pin = EN_BQ_Pin|EN_12LED_Pin|EN_3_3V_Pin|EN_6V_Pin
                          |UART_DE_Pin|EN_water_sens_Pin|Light_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_RS_Pin EN_5V_Pin PB15 EN_Hall_Pin */
  GPIO_InitStruct.Pin = EN_RS_Pin|EN_5V_Pin|GPIO_PIN_15|EN_Hall_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : water_sens_Pin */
  GPIO_InitStruct.Pin = water_sens_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(water_sens_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : go_to_sleep_Pin */
  GPIO_InitStruct.Pin = go_to_sleep_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(go_to_sleep_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//------------------------------------------------
//              Формирование CRC16              //
//------------------------------------------------
uint8_t GenCRC16(uint8_t* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;

	buff[len++] = lo;
	buff[len++] = hi;
	return len;
}
//------------------------------------------------
//              Проверка CRC16                  //
//------------------------------------------------
uint8_t CheckCRC16(uint8_t* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len - 2; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;
	if ((buff[len - 2] == lo) &&
		(buff[len - 1] == hi))
	{
		return 1;
	}
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
  {
      //Прием байт по modbus
    if(huart == &huart1) {
        HAL_TIM_Base_Stop_IT(&htim6);
        __HAL_TIM_SetCounter(&htim6, 0);
        HAL_TIM_Base_Start_IT(&htim6);
        buff_uart[cnt] = str;
        cnt = cnt + 1;
        HAL_UART_Receive_IT(&huart1, &str, 1);
    }
  }
  
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
      // проверка окончания транзакция по modbus
    if(htim->Instance == TIM6) //check if the interrupt comes from TIM6
    {
        uint16_t addr_var = 0;
        uint16_t num_word = 0;
        
        if (cnt > 4) {
            HAL_TIM_Base_Stop_IT(&htim6);
            __HAL_TIM_SetCounter(&htim6, 0); // сброс таймера
            
            if (buff_uart[0] == ID_STM32 && CheckCRC16(buff_uart, cnt)) {
                addr_var = (buff_uart[2] << 8) |  buff_uart[3];
                num_word = (buff_uart[4] << 8) |  buff_uart[5];
                if (buff_uart[1] == 0x03) { // чтение
                    for(int i = 0; i < num_word; i++) {                    
                        switch (addr_var) {
                            case 0x0001: // чтение темпы
                                buff_uart[i + 3] = (temp >> 8) & 0xFF;
                                buff_uart[i + 4] = temp & 0xFF;
                                break;
                            case 0x0002: // еще переменная
                                buff_uart[i + 3] = (temp >> 8) & 0xFF;
                                buff_uart[i + 4] = temp & 0xFF;
                                break;
                            default:
                                //ошибка
                              break;
                        }
                        addr_var = addr_var + 1;
                    }
                    
                    buff_uart[2] = ((buff_uart[2] << 8) |  buff_uart[3]) * 2;
                    cnt = GenCRC16(buff_uart, buff_uart[2] + 3);
                
                } else if (buff_uart[1] == 0x06) { // запись
                    //запись в в регистр
                }
            
            }
            
            HAL_GPIO_WritePin(GPIOA, UART_DE_Pin, GPIO_PIN_SET); // активация передачи
            HAL_UART_Transmit_IT(&huart1, buff_uart, cnt);

            cnt = 0;
        } else {
            cnt = 0;
        }     
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart1) {
      HAL_GPIO_WritePin(GPIOA, UART_DE_Pin, GPIO_PIN_RESET);
      HAL_UART_Receive_IT(&huart1, &str, 1);
      cnt = 0;
    }
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
