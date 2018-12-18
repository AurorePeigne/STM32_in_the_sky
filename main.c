
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uint16_t conv_temp[1]={0x00};
uint16_t conv_hum;

//variable WhoIAm
uint8_t sub_addr_who_am_i[1]={0x0F};
uint8_t who_am_i[1]={0x00};

//Variable Temperature
uint8_t sub_addr_TEMP_OUT_L[1]={0x2A};
uint8_t TEMP_OUT_L[1]={0x00};
uint8_t sub_addr_TEMP_OUT_H[1]={0x2B};
uint8_t TEMP_OUT_H[1]={0x00};

//variable conversion temperature
uint8_t sub_addr_T0_degC_x8[1]={0x32};
uint8_t sub_addr_T1_degC_x8[1]={0x33};
uint8_t T0_degC_x8[1]={0x00};
uint8_t T1_degC_x8[1]={0x00};

uint8_t T1T0msb[1]={0x35};
uint8_t sub_addr_T0_OUT_L[1]={0x00};
uint8_t sub_addr_T0_OUT_H[1]={0x00};
uint8_t T0_OUT_L[1]={0x3C};
uint8_t T0_OUT_H[1]={0x3D};

uint8_t sub_addr_T1_OUT_L[1]={0x00};
uint8_t sub_addr_T1_OUT_H[1]={0x00};
uint8_t T1_OUT_L[1]={0x3E};
uint8_t T1_OUT_H[1]={0x3F};


//variables Humidity
uint8_t sub_addr_HUMIDITY_OUT_L[1]={0x28};
uint8_t HUMIDITY_OUT_L[1]={0x00};
uint8_t sub_addr_HUMIDITY_OUT_H[1]={0x29};
uint8_t HUMIDITY_OUT_H[1]={0x00};

// variable conversion humidité
uint8_t sub_addr_H0_rH_x2[1]={0x30};
uint8_t sub_addr_H1_rH_x2[1]={0x31};
uint8_t H0_rH_x2[1]={0x00};
uint8_t H1_rH_x2[1]={0x00};

uint8_t sub_addr_H0_T0_OUT_L[1]={0x00};
uint8_t sub_addr_H0_T0_OUT_H[1]={0x00};
uint8_t H0_T0_OUT_L[1]={0x36};
uint8_t H0_T0_OUT_H[1]={0x37};

uint8_t sub_addr_H1_T0_OUT_L[1]={0x00};
uint8_t sub_addr_H1_T0_OUT_H[1]={0x00};
uint8_t H1_T0_OUT_L[1]={0x3A};
uint8_t H1_T0_OUT_H[1]={0x3B};

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

			// WHO_I_AM
			while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_who_am_i, 1, HAL_TIMEOUT) != HAL_OK);
			while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, who_am_i, 1, HAL_TIMEOUT) != HAL_OK);
			HAL_Delay(1000);

		  //Temperature
		  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_TEMP_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
		  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, TEMP_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
		  HAL_Delay(1000);
		  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_TEMP_OUT_L, 1, HAL_TIMEOUT)!= HAL_OK);
		  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, TEMP_OUT_L, 1, HAL_TIMEOUT)!= HAL_OK);
		  HAL_Delay(1000);

		  uint16_t TEMP_OUT = (TEMP_OUT_H[1]<<8) +  TEMP_OUT_L[1];

	  	  //CONVERSION Temperature

	  	  	  //T0_degC_x8 et T1_degC_x8
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_T0_degC_x8, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, T0_degC_x8, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_T1_degC_x8, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, T1_degC_x8, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	    //division par 8
	  	  uint16_t T0_degC = T0_degC_x8[1]<<8;
	  	  uint16_t T1_degC = T1_degC_x8[1]<<8;
	  	  	  //T0_OUT
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_T0_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, T0_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_T0_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, T0_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  uint16_t T0_OUT = (T0_OUT_H[1]<<8) +  T0_OUT_L[1];
	    	  //T1_OUT
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_T1_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, T1_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_T1_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, T1_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  uint16_t T1_OUT = (T1_OUT_H[1]<<8) +  T1_OUT_L[1];


	  	  //Compute the temperature
	  	  //int T= ( ((T1_degC - T0_degC)(TEMP_OUT - T0_OUT))/(T1_OUT - T0_OUT) +  T0_degC );

		  //HUMIDITY
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_HUMIDITY_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, HUMIDITY_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_HUMIDITY_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, HUMIDITY_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);

	  	  uint16_t HUM_OUT = (HUMIDITY_OUT_H[1]<<8) +  HUMIDITY_OUT_L[1];

	  	  //CONVERSION Humidity

	  	 	 //H0_rH_x2 et H1_rH_x2
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_H0_rH_x2, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, H0_rH_x2, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_H1_rH_x2, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, H1_rH_x2, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	    //division par 2
	  	  uint16_t H0_rH = H0_rH_x2[1]<<2;
	  	  uint16_t H1_rH = H1_rH_x2[1]<<2;
	  	  	  //H0_T0_OUT
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_H0_T0_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, H0_T0_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_H0_T0_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, H0_T0_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  uint16_t H0_T0_OUT = (H0_T0_OUT_H[1]<<8) +  H0_T0_OUT_L[1];
	  	  	  //H1_T0_OUT
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_H1_T0_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, H1_T0_OUT_H, 1, HAL_TIMEOUT) != HAL_OK);
	  	  HAL_Delay(1000);
	  	  while(HAL_I2C_Master_Transmit(&hi2c1, 0xBE, sub_addr_H1_T0_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	  	  while(HAL_I2C_Master_Receive(&hi2c1, 0xBE, H1_T0_OUT_L, 1, HAL_TIMEOUT) != HAL_OK);
	   	  HAL_Delay(1000);
	  	  uint16_t H1_T0_OUT = (H1_T0_OUT_H[1]<<8) +  H1_T0_OUT_L[1];

	  	  //Compute the humidity
	  	 // int H= ( ((H1_rH - H0_rH)( HUM_OUT - H0_T0_OUT))/(H1_T0_OUT - H0_T0_OUT) + H0_rH  );


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
