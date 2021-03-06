
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32l4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdlib.h"
#include "I2C_stack.h"
#include "lora_fonction.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
 void SystemClock_Config(void);


 static void MX_I2C1_Init(void)
 {

   hi2c1.Instance = I2C1;
   hi2c1.Init.Timing = 0x10909CEC;
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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */


  TEMPandHUM_init();
  PRES_init();

 HAL_Delay(100);






/*
LORA_AT_APPEUI_SET("900dcafe00000001",16,4);//001
MX_LPUART1_UART_Init();
*/










 LORA_AT_SET("AT+APPEUI=70b3d57ed0014d9e");//APPEUI=900dcafe00000001");//last = 1
 HAL_UART_DeInit(&hlpuart1);
 HAL_UART_Init(&hlpuart1);

 LORA_AT_SET("AT+ASK=9a2e35347a2da3c76f583e31db2fadab");//APPEUI=900dcafe00000001");//last = 1
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);



  LORA_AT_SET("AT+NSK=1a47ad204af7bba26a11acd46a43d5c7");//APPEUI=900dcafe00000001");//last = 1
   HAL_UART_DeInit(&hlpuart1);
   HAL_UART_Init(&hlpuart1);


/*
 LORA_AT_GET("AT+EUI",42);
 HAL_UART_DeInit(&hlpuart1);
 HAL_UART_Init(&hlpuart1);



 LORA_AT_GET("AT+APPEUI",42);
 HAL_UART_DeInit(&hlpuart1);
 HAL_UART_Init(&hlpuart1);
*/

/*
 LORA_AT_SET("AT+AK=886f53be9f8142a1fccA26985ffc8b98");//9a2e35347a2da3c76f583e31db2fadab   886f53be9f8142a1fccA26985ffc8b98
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);

  */


  LORA_AT_SET("AT+DC=0");//APPEUI=900dcafe00000001");//last = 1
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);




  LORA_AT_SET("AT+ADDR=26011a95");//APPEUI=900dcafe00000001");//last = 1
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);






LORA_AT_JOIN_SET(0);
HAL_UART_DeInit(&hlpuart1);
HAL_UART_Init(&hlpuart1);



		 get_TEMP();
		 get_HUM();
		 get_PRES();

char dest1[4];
char dest2[4];
char dest3[4];
HAL_Delay(100);
LORA_AT_SEND(CONV_CHAR(temp16[0],dest1),CONV_CHAR(hum16[0],dest2),12,5);
HAL_UART_DeInit(&hlpuart1);
HAL_UART_Init(&hlpuart1);

    	while(1){

    		LORA_AT_SEND(CONV_CHAR(temp16[0],dest1),CONV_CHAR(hum16[0],dest2),12,5);
    		HAL_UART_DeInit(&hlpuart1);
    		HAL_UART_Init(&hlpuart1);
      		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      		HAL_Delay(100);

      	}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

      	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  //  0x65 0x84
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

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
