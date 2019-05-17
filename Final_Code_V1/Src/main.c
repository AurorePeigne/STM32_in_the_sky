/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "sdmmc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SENSORS_V6.h"
#include "string.h"
#include "gps.h"
#include "lora_fonction.h"
#include "flash.h"
#include "stm32l4xx_hal_flash_ex.h"

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

/* USER CODE BEGIN PV */
uint8_t aShowTime[50] = {0};
uint8_t toggle=0;
uint64_t erreur = 0x00;
uint8_t i = 0;
uint32_t erreur_data = 0x00;
uint32_t erreur_write = 0x00;

uint32_t *ptr2=ADDR_FLASH_PAGE_19;

uint32_t cpt_flash = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void RTC_TimeShow(uint8_t* showtime)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char*)showtime,"%02d:%02d:%02d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  HAL_UART_Transmit(&huart2,showtime,50,100);
}



static void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;



  /* Enable PWR clock */
  __HAL_RCC_PWR_CLK_ENABLE();
}

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //*****************************INIT SENSORS*************************//

  TEMP_HUM_init();
  PRES_init();
 // MAGN_init();
  //DMA_init();

HAL_Delay(10);

  uint32_t GPS_COORD[3];

//  uint32_t adr_stop_2=ADDR_FLASH_PAGE_19+ FLASH_PAGE_SIZE - 1;
//
//
//  uint64_t skip_temp=0;
//
//  uint64_t skip;
//
//  EraseFlash(ADDR_FLASH_PAGE_18,ADDR_FLASH_PAGE_19);
//WriteFlash(ADDR_FLASH_PAGE_18, skip_temp, ADDR_FLASH_PAGE_19);
//uint32_t *ptr=ADDR_FLASH_PAGE_17;
//skip=*ptr;

  //*****************************INIT LORA****************************//
//if (skip!=1){
//	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//
//	skip_temp++;
//	skip=skip_temp;
//	 EraseFlash(ADDR_FLASH_PAGE_17,ADDR_FLASH_PAGE_18);
//	WriteFlash(ADDR_FLASH_PAGE_17, skip, ADDR_FLASH_PAGE_18);

/*

   LORA_AT_SET("AT+ATZ");
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);

//OTAA


  LORA_AT_SET("AT+APPEUI=db815fb2f1feaf0c");//AT+APPEUI=70b3d57ed0014d9e ABP
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);



  LORA_AT_SET("AT+AK=fc72a6df7236532872eb1aeeccd0e306");//OTAA
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);



  //OTAA FINI




*/






  LORA_AT_SET("AT+ATZ");
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);

  LORA_AT_SET("AT+APPEUI=70b3d57ed0014d9e");//APPEUI=900dcafe00000001");//last = 1
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);

  LORA_AT_SET("AT+ASK=9a2e35347a2da3c76f583e31db2fadab");
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);

  LORA_AT_SET("AT+NSK=1a47ad204af7bba26a11acd46a43d5c7");
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);


  LORA_AT_SET("AT+ADDR=26011a95");
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);



  LORA_AT_SET("AT+DC=0");
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);



  LORA_AT_JOIN_SET(0);
  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);


//}

//LORA_AT_SET("AT+EUI");
//HAL_UART_DeInit(&hlpuart1);
//HAL_UART_Init(&hlpuart1);


  //*****************************END INIT LORA****************************/


  //*****************************SEND THROUGH LORA***********************//
 // HAL_PWREx_EnableSRAM2ContentRetention();



  char buff_lon[20];
  char buff_lat[20];
  char buff_alt[20];
  char buff_temp[20];
  char buff_hum[20];
  //LORA_AT_SEND(CONV_CHAR32(GPS_COORD[0],buff_lon),CONV_CHAR32(GPS_COORD[1],buff_lat),CONV_CHAR32(GPS_COORD[2],buff_alt));

  //*****************************END SEND THROUGH LORA*******************//

  HAL_UART_DeInit(&hlpuart1);
  HAL_UART_Init(&hlpuart1);


  //*****************************FLASH**********************************//
  uint64_t cpt;
uint32_t adr_stop=ADDR_FLASH_PAGE_20+ FLASH_PAGE_SIZE - 1;
/*
  uint64_t cpt=0;
  EraseFlash(ADDR_FLASH_PAGE_19,ADDR_FLASH_PAGE_20);
erreur_write=  WriteFlash(ADDR_FLASH_PAGE_19, cpt, adr_stop );

*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  RTC_TimeShow(aShowTime);
		  if (toggle==1){


			  get_TEMP();				//TEMPERATURE		(°C)
			  get_HUM();				//HUMIDITE			(%)
			  get_PRES();				//PRESSION			(mbar ou hPa)
			  get_MAGN();				//CHAMP MAGNETIQUE	(mgauss)
			  get_ADC();				//GAS				(ppm)
			  GPS_GETPOS(GPS_COORD);	//POSITION GPS		(m)



			  cpt=*ptr2;
			  cpt++;

  EraseFlash(ADDR_FLASH_PAGE_19,ADDR_FLASH_PAGE_20);
WriteFlash(ADDR_FLASH_PAGE_19, cpt, ADDR_FLASH_PAGE_20);


			 LORA_AT_SEND(CONV_CHAR32(GPS_COORD[0],buff_lon),CONV_CHAR32(GPS_COORD[1],buff_lat),CONV_CHAR32(GPS_COORD[2],buff_alt),CONV_CHAR(temp16[0],buff_temp), CONV_CHAR(hum16[0],buff_hum));
			//  SD_SENSORS(GPS_COORD[0],GPS_COORD[1],GPS_COORD[2]);



						if((cpt)%5 == 0)
			 			  	   {
			 			  		  	erreur = EraseFlash(FLASH_USER_START_ADDR,ADDR_FLASH_PAGE_255);
			 			  		  	erreur_write= WriteFlash(FLASH_USER_START_ADDR, DATA_64, FLASH_USER_END_ADDR);
			 			  		  	//  erreur_data=ReadFlash(FLASH_USER_START_ADDR, DATA_32);

			 			  		  	//  erreur_data = algo_flash_test(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR, DATA_32, DATA_64);
			 			  		  	cpt_flash++;

			 			  	   }



/* ************* LOW POWER MANAGEMENT ******************/
/* Clear all related wakeup flags */
			//  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			  toggle=0;
			  MX_RTC_Init();
			  HAL_PWR_EnterSTANDBYMode();


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	//toggle a variable to notify we can exit low power mode and execute the routine
toggle=1;
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
