
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
#include "stm32l0xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//TEMPERATURE & HUMIDITY SENSOR
uint8_t addr_sensor[1]={0xBE};

//TEMPERATURE
uint8_t addr_T0_degC_x8[1]={0x32};
uint8_t T0_degC_x8[1]={0x00};
uint8_t addr_T1_degC_x8[1]={0x33};
uint8_t T1_degC_x8[1]={0x00};

uint8_t addr_MSB[1]={0x35};
uint8_t MSB[1]={0x00};

int16_t T0_degC_x8_u16[1]={0x00};;
int16_t T1_degC_x8_u16[1]={0x00};;
int16_t T0_DegC[1]={0x00};
int16_t T1_DegC[1]={0x00};


uint8_t addr_T0_OUT_L[1]={0x3C};
uint8_t T0_OUT_L[1]={0x00};
uint8_t addr_T0_OUT_H[1]={0x3D};
uint8_t T0_OUT_H[1]={0x00};
int16_t T0_OUT[1]={0x00};

uint8_t addr_T1_OUT_L[1]={0x3E};
uint8_t T1_OUT_L[1]={0x00};
uint8_t addr_T1_OUT_H[1]={0x3F};
uint8_t T1_OUT_H[1]={0x00};
int16_t T1_OUT[1]={0x00};

uint8_t addr_T_OUT_L[1]={0x2A};
uint8_t T_OUT_L[1]={0x00};
uint8_t addr_T_OUT_H[1]={0x2B};
uint8_t T_OUT_H[1]={0x00};
int16_t T_OUT[1]={0x00};


int32_t temp32;
int16_t temp_value;


//HUMIDITY
uint8_t H0_rH_x2[1]={0x00};
uint8_t addr_H0_rH_x2[1]={0x30};

uint8_t H1_rH_x2[1]={0x00};
uint8_t addr_H1_rH_x2[1]={0x31};
int16_t H0_rH;
int16_t H1_rH;

int16_t H_OUT;

uint8_t H0_T0_OUT_L[1]={0x00};
uint8_t addr_H0_T0_OUT_L[1]={0x36};

uint8_t H0_T0_OUT_H[1]={0x00};
uint8_t addr_H0_T0_OUT_H[1]={0x37};

int16_t H0_T0_OUT;

uint8_t H1_T0_OUT_L[1]={0x00};
uint8_t addr_H1_T0_OUT_L[1]={0x3A};

uint8_t H1_T0_OUT_H[1]={0x00};
uint8_t addr_H1_T0_OUT_H[1]={0x3B};

int16_t H1_T0_OUT;

uint8_t H_T0_OUT_L[1]={0x00};
uint8_t addr_H_T0_OUT_L[1]={0x28};

uint8_t H_T0_OUT_H[1]={0x00};
uint8_t addr_H_T0_OUT_H[1]={0x29};

int16_t H_T0_OUT;

int32_t hum;

int16_t value_hum;

//PRESSURE SENSOR
uint8_t addr_sensor_press[1]={0xBA};
//uint8_t addr_CTRL_REG2[1]={0x11}; //METTRE A 1 ?

uint8_t addr_REF_P_H[1]={0x17};
uint8_t REF_P_H[1]={0x00};
uint8_t addr_REF_P_L[1]={0x16};
uint8_t REF_P_L[1]={0x00};
uint8_t addr_REF_P_XL[1]={0x15};
uint8_t REF_P_XL[1]={0x00};

int32_t REF_P;

uint8_t addr_PRESS_OUT_H[1]={0x2A};
uint8_t P_OUT_H[1]={0x00};
uint8_t addr_PRESS_OUT_L[1]={0x29};
uint8_t P_OUT_L[1]={0x00};
uint8_t addr_PRESS_OUT_XL[1]={0x28};
uint8_t P_OUT_XL[1]={0x00};

int32_t P_OUT;

int32_t press32;
int16_t value_press;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void get_register(uint8_t addr_sensor[], uint8_t addr_register[], uint8_t data_register[]);
void get_temp(void);
void get_hum(void);
void get_press(void);
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

	  get_temp();

	  get_hum();

	  get_press();




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

/* USER CODE BEGIN 4 */
void get_register(uint8_t addr_sensor[], uint8_t addr_register[], uint8_t data_register[]){

	  while(HAL_I2C_Master_Transmit(&hi2c1, addr_sensor[0], addr_register, 1, HAL_TIMEOUT) != HAL_OK);
	  while(HAL_I2C_Master_Receive(&hi2c1, addr_sensor[0], data_register, 1, HAL_TIMEOUT) != HAL_OK);
	  HAL_Delay(100);
}

void get_temp(void){

	// read register T0_degC_x8
	get_register(addr_sensor, addr_T0_degC_x8, T0_degC_x8);
	// read register T1_degC_x8
	get_register(addr_sensor, addr_T1_degC_x8, T1_degC_x8);
	// Read register MSB
	get_register(addr_sensor, addr_MSB, MSB);

	T0_degC_x8_u16[0] = (((uint16_t)(MSB[0] & 0x03)) << 8) + ((uint16_t)T0_degC_x8[0]);

	T1_degC_x8_u16[0] = (((uint16_t)(MSB[0] & 0x0C)) << 6) + ((uint16_t)T1_degC_x8[0]);

	T0_DegC[0] = T0_degC_x8_u16[0] >> 3;

	T1_DegC[0] = T1_degC_x8_u16[0] >> 3;

	// Read register T0_OUT_L
	get_register(addr_sensor, addr_T0_OUT_L, T0_OUT_L);
	// Read register T0_OUT_H
	get_register(addr_sensor, addr_T0_OUT_H, T0_OUT_H);

	  T0_OUT[0] = (((uint16_t)T0_OUT_H[0])<<8) + (uint16_t)T0_OUT_L[0];

	// Read register T1_OUT_L
	get_register(addr_sensor, addr_T1_OUT_L, T1_OUT_L);
	// Read register T1_OUT_H
	get_register(addr_sensor, addr_T1_OUT_H, T1_OUT_H);

	T1_OUT[0] = (((uint16_t)T1_OUT_H[0])<<8) + (uint16_t)T1_OUT_L[0];

	// Read register T_OUT_L
	get_register(addr_sensor, addr_T_OUT_L, T_OUT_L);
	// Read register T_OUT_H
	get_register(addr_sensor, addr_T_OUT_H, T_OUT_H);


	  T_OUT[0]=(((uint16_t)T_OUT_H[0])<<8) + (uint16_t)T_OUT_L[0];

	  temp32= (int32_t)(T1_DegC[0]-T0_DegC[0])*(T_OUT[0]-T0_OUT[0]);
	  temp_value = (int16_t)(temp32/(T1_OUT[0] - T0_OUT[0])) + T0_DegC[0];

}


void get_hum(void){

	get_register(addr_sensor, addr_H0_rH_x2, H0_rH_x2);

	get_register(addr_sensor, addr_H1_rH_x2, H1_rH_x2);

	H0_rH= H0_rH_x2[0]>>2;
	H1_rH= H1_rH_x2[0]>>2;

	get_register(addr_sensor, addr_H0_T0_OUT_L, H0_T0_OUT_L);

	get_register(addr_sensor, addr_H0_T0_OUT_H, H0_T0_OUT_H);

	H0_T0_OUT = (H0_T0_OUT_H[0]<<8) + H0_T0_OUT_L[0];


	get_register(addr_sensor, addr_H1_T0_OUT_L, H1_T0_OUT_L);

	get_register(addr_sensor, addr_H1_T0_OUT_H, H1_T0_OUT_H);

	H1_T0_OUT = (H1_T0_OUT_H[0]<<8) + H1_T0_OUT_L[0];


	get_register(addr_sensor, addr_H_T0_OUT_L, H_T0_OUT_L);

	get_register(addr_sensor, addr_H_T0_OUT_H, H_T0_OUT_H);

	H_T0_OUT = (H_T0_OUT_H[0]<<8) + H_T0_OUT_L[0];

	hum = (H1_rH-H0_rH)*(H_T0_OUT-H0_T0_OUT);
	value_hum = (hum/(H1_T0_OUT-H0_T0_OUT))+H0_rH;

}

void get_press(void){
	get_register(addr_sensor_press, addr_REF_P_H, REF_P_H);
	get_register(addr_sensor_press, addr_REF_P_L, REF_P_L);
	get_register(addr_sensor_press, addr_REF_P_XL, REF_P_XL);

	REF_P =  ( ((REF_P_H[0]+0x10)<<16) + (REF_P_L[0]<<8) + (REF_P_XL[0]) );

	get_register(addr_sensor_press, addr_PRESS_OUT_H, P_OUT_H);
	get_register(addr_sensor_press, addr_PRESS_OUT_L, P_OUT_L);
	get_register(addr_sensor_press, addr_PRESS_OUT_XL, P_OUT_XL);

	//Avec OFFSET Manuel
	//P_OUT = ( ((P_OUT_H[0]+0x10)<<16) + (P_OUT_L[0]<<8) + (P_OUT_XL[0]) );
	//Sans offset
	P_OUT = ( ((P_OUT_H[0])<<16) + (P_OUT_L[0]<<8) + (P_OUT_XL[0]) );
	//Probleme d'offset : verifier avec la valeur de reference si elle n'en est pas a l'origine
	press32 = P_OUT + REF_P;

	value_press = press32/4096;

}
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
