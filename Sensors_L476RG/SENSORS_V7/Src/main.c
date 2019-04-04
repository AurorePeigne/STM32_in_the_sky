/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  *		A TESTER
  * 	PIN PA6 receive tension of battery be careful : 5V max !
  *
  *	    printf : run debug SWV
  * 	Useful with putty; check COM put sérial num COM & give name au pif in saved sessions, flow control:no; go in serial -> choose COM & baud come back init save
  *	    printf
  *
  *	    add IKSO1A2 : Temperature Humidity Pressure Accelerometer Magnetometer
  *
  *	    Print all 5 secs :
  *	    Sensors
  *	    ----------------------------
  *		SENSOR Gaz : 3570
  *		Temperature :241
  *		Humidity :47
  *		Pressure :1028
  *		----------------------------
  *
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_TIMEOUT 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//ADC for battery
static  float ADC_battery =0;
//I2C
uint8_t sub_addr_who_am_i[1]={0x0F}; //Who AM I
uint8_t who_am_i[1]={0x00};

//TEMPERATURE/////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t addr_T0_degC_x8[1]={0x32};
uint8_t addr_T1_degC_x8[1]={0x33};
uint8_t T0_degC_x8[1];
uint8_t T1_degC_x8[1];
uint16_t T0_degC_DIV8[1];
uint16_t T1_degC_DIV8[1];
uint16_t T0_degC[1];
uint16_t T1_degC[1];

uint8_t addr_T0_T1_msb[1]={0x35};
uint8_t T0_T1_msb[1];
uint8_t T0_msb[1];
uint8_t T1_msb[1];

uint8_t addr_T0_H[1]={0x3D};
uint8_t addr_T0_L[1]={0x3C};
uint8_t T0_H[1];
uint8_t T0_L[1];

uint8_t addr_T1_H[1]={0x3F};
uint8_t addr_T1_L[1]={0x3E};
uint8_t T1_H[1];
uint8_t T1_L[1];

uint8_t addr_T_OUT_H[1]={0x2B};
uint8_t addr_T_OUT_L[1]={0x2A};
uint8_t T_OUT_H[1];
uint8_t T_OUT_L[1];

uint16_t T0_OUT[1];
uint16_t T1_OUT[1];
uint16_t T_OUT[1];

uint8_t addr_CTRL_REG1_TEMP[1]={0x20};
uint8_t CTRL_REG1_TEMP[1]={0x83};

uint16_t temp16[1];

//HUMIDITY////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t addr_HUM_H[1]={0x29};
uint8_t addr_HUM_L[1]={0x28};
uint8_t HUM_H[1];
uint8_t HUM_L[1];

uint8_t addr_H0_rH_x2[1]={0x30};
uint8_t addr_H1_rH_x2[1]={0x31};
uint8_t H0_rH_x2[1];
uint8_t H1_rH_x2[1];

uint8_t addr_H0_T0_OUT_L[1]={0x36};
uint8_t addr_H0_T0_OUT_H[1]={0x37};
uint8_t H0_T0_OUT_L[1];
uint8_t H0_T0_OUT_H[1];

uint8_t addr_H1_T0_OUT_L[1]={0x3A};
uint8_t addr_H1_T0_OUT_H[1]={0x3B};
uint8_t H1_T0_OUT_L[1];
uint8_t H1_T0_OUT_H[1];

uint16_t H0_rH[1];
uint16_t H1_rH[1];
uint16_t H0[1];
uint16_t H1[1];
uint16_t H_OUT[1];

uint16_t hum16[1];

//PRESSURE/////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t addr_PRES_OUT_XL[1]={0x28};
uint8_t addr_PRES_OUT_L[1]={0x29};
uint8_t addr_PRES_OUT_H[1]={0x2A};

uint8_t addr_REF_P_XL[1]={0x15};
uint8_t addr_REF_P_L[1]={0x16};
uint8_t addr_REF_P_H[1]={0x17};

uint8_t PRES_OUT_XL[1];
uint8_t PRES_OUT_L[1];
uint8_t PRES_OUT_H[1];

uint8_t REF_P_XL[1];
uint8_t REF_P_L[1];
uint8_t REF_P_H[1];

uint32_t PRES_x4096[1];
uint32_t REF_P_x4096[1];

uint8_t addr_CTRL_REG1_PRES[1]={0x10};
uint8_t CTRL_REG1_PRES[1]={0x50};

uint8_t addr_CTRL_REG2_PRES[1]={0x11};
uint8_t CTRL_REG2_PRES[1]={0x01};

uint8_t addr_INTERRUPT_CFG[1]={0x0B};
uint8_t INTERRUPT_CFG[1]={0x30};

uint32_t pres32[1];

//MAGNETOMETER//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t addr_CFG_REG_A_M[1]={0x60};
uint8_t CFG_REG_A_M[1]={0x0C};

uint8_t addr_OUTX_L_REG_M[1]={0x68};
uint8_t addr_OUTX_H_REG_M[1]={0x69};
uint8_t addr_OUTY_L_REG_M[1]={0x6A};
uint8_t addr_OUTY_H_REG_M[1]={0x6B};
uint8_t addr_OUTZ_L_REG_M[1]={0x6C};
uint8_t addr_OUTZ_H_REG_M[1]={0x6D};

uint8_t OUTX_L_REG_M[1];
uint8_t OUTX_H_REG_M[1];
uint8_t OUTY_L_REG_M[1];
uint8_t OUTY_H_REG_M[1];
uint8_t OUTZ_L_REG_M[1];
uint8_t OUTZ_H_REG_M[1];

uint16_t magnX16[1];
uint16_t magnY16[1];
uint16_t magnZ16[1];

uint16_t magn16[1];

//ACCELEROMETER/////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t addr_CTRL_REG1_ACCEL[1]={0x20};
uint8_t CTRL_REG1_ACCEL[1]={0x97};

uint8_t addr_X_LI_L[1]={0x28};
uint8_t addr_X_LI_H[1]={0x29};
uint8_t addr_Y_LI_L[1]={0x2A};
uint8_t addr_Y_LI_H[1]={0x2B};
uint8_t addr_Z_LI_L[2]={0x2C};
uint8_t addr_Z_LI_H[2]={0x2D};

uint8_t X_LI_L[1];
uint8_t X_LI_H[1];
uint8_t Y_LI_L[1];
uint8_t Y_LI_H[1];
uint8_t Z_LI_L[1];
uint8_t Z_LI_H[1];

uint16_t accelX16[1];
uint16_t accelY16[1];
uint16_t accelZ16[1];

uint8_t cpt=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//ADC avec TIM3
void ADC_Init(void); //Read ADC
//I2C
uint16_t est_negatif(uint16_t valeur);
uint16_t concatenate(uint8_t valeur_H, uint8_t valeur_L);
uint16_t div8(uint16_t valeur);
uint16_t div2(uint16_t valeur);
void Who_am_i(void); //Send who_am_i data; must send
void Register(void); //Update data for temperature and humidity
void temperature_IKSO1A2(void);//give temperature in celcius
void humidity_IKSO1A2(void);//give humidity in percentage
void pressure_IKSO1A2(void);//give pressure in bar
void INIT_MAGNETO_LSM6DSL(void);
void MAGNETO_LSM6DSL(void);//give magnetic atmosphere
void INIT_ACCELERO_LSM6DSL(void);
void ACCELERO_LSM6DSL(void);//give accelerometer data
void READ_BATTERY_VOLTAGE(void);//give value of battery voltage
//void MX_ADC1_CH_11_Init(void);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("\033[2J\033[1;1H"); //Affiche seulement les nouvelles valeurs à virer pour le final
  printf("Sensors\r\n");
  Register();
  ADC_Init();
  HAL_TIM_Base_Start_IT(&htim3);//Donne base temps -> tous les 5 sec
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
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

//printf
int _write(int file,char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
    return len;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) //Récupérer la valeur de l'ADC une fois l’IT acquittée
{

	ADC_battery = HAL_ADC_GetValue(AdcHandle);
	READ_BATTERY_VOLTAGE();
	temperature_IKSO1A2();
	humidity_IKSO1A2();
	pressure_IKSO1A2();
	printf("----------------------------\r\n");
	cpt++;

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_ADC_Start_IT(&hadc1);

}
void ADC_Init() //Lancer l'ADC
{
	if (HAL_ADC_Start_IT(&hadc1)!= HAL_OK)
	{
		Error_Handler();
	}
}
void READ_BATTERY_VOLTAGE()
{
	ADC_battery= floor(10*(ADC_battery/558.7f) +0.5)/10 ;//conversion au dixieme près : 5V->2821 valeur sortie
	printf("Voltage : %f\r\n", ADC_battery);
}
//I2C
uint16_t est_negatif(uint16_t valeur){
if (valeur>=0x8000){
		valeur=~valeur+1;
	}
return valeur;
}

uint16_t concatenate(uint8_t valeur_H, uint8_t valeur_L){
	uint16_t valeur = (valeur_H<<8) + valeur_L;
	return valeur;
}

uint16_t div8(uint16_t valeur){
	uint16_t result = (valeur>>3);
	return result;
}

uint16_t div2(uint16_t valeur){
	uint16_t result = (valeur>>1);
	return result;
}
void Who_am_i()
{
	  HAL_I2C_Mem_Read(&hi2c1,0xBF, sub_addr_who_am_i[0], 1, who_am_i, 1, I2C_TIMEOUT);
	  printf("Who AM I : %d", who_am_i[0]);
}
void Register()
{
	//Temperature & humidity
	HAL_I2C_Mem_Write(&hi2c1,0xBE, addr_CTRL_REG1_TEMP[0], 1, CTRL_REG1_TEMP, 1, I2C_TIMEOUT);
	//PRESSURE
	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_CTRL_REG1_PRES[0], 1, CTRL_REG1_PRES, 1, I2C_TIMEOUT);

	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_CTRL_REG2_PRES[0], 1, CTRL_REG2_PRES, 1, I2C_TIMEOUT);

	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_INTERRUPT_CFG[0], 1, INTERRUPT_CFG, 1, I2C_TIMEOUT);

}
//Temperature
void temperature_IKSO1A2()
{
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T0_degC_x8[0], 1, T0_degC_x8, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T1_degC_x8[0], 1, T1_degC_x8, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T0_T1_msb[0], 1, T0_T1_msb, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T0_H[0], 1, T0_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T0_H[0], 1, T0_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T0_L[0], 1, T0_L, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T1_H[0], 1, T1_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T1_L[0], 1, T1_L, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T_OUT_H[0], 1, T_OUT_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T_OUT_L[0], 1, T_OUT_L, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_T_OUT_L[0], 1, T_OUT_L, 1, I2C_TIMEOUT);

		//CONCATENATION
		T0_OUT[0]	= (T0_H[0]<<8) + T0_L[0];
		T1_OUT[0]	= (T1_H[0]<<8) + T1_L[0];
		T_OUT[0]	= (T_OUT_H[0]<<8) + T_OUT_L[0];

		//GESTION VALEURS NEGATIVES
		est_negatif(T0_OUT[0]);
		est_negatif(T1_OUT[0]);
		est_negatif(T_OUT[0]);

		//CALCUL DES T0_degC ET T1_degC FINALES
		T0_msb[0]		= T0_T1_msb[0] & 0x3;
		T1_msb[0]		= (T0_T1_msb[0] & 0xC)>>2;
		T0_degC[0] 		= (T0_msb[0]<<8) + T0_degC_x8[0];
		T1_degC[0] 		= (T1_msb[0]<<8) + T1_degC_x8[0];
		T0_degC_DIV8[0]	= T0_degC[0]>>3;
		T1_degC_DIV8[0]	= T1_degC[0]>>3;

		//CALCUL DE LA TEMPERATURE
		temp16[0] = ((int16_t)(T_OUT[0]-T0_OUT[0]))*10*((int16_t)(T1_degC_DIV8[0]-T0_degC_DIV8[0]))/((int16_t)(T1_OUT[0]-T0_OUT[0]))+(int16_t)(T0_degC_DIV8[0])*10;
		printf("Temperature :%d\r\n", temp16[0]);
}
//Humidity
void humidity_IKSO1A2()
{
	    HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_HUM_H[0], 1, HUM_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_HUM_L[0], 1, HUM_L, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_H0_rH_x2[0], 1, H0_rH_x2, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_H1_rH_x2[0], 1, H0_rH_x2, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_H0_T0_OUT_H[0], 1, H0_T0_OUT_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_H0_T0_OUT_L[0], 1, H0_T0_OUT_L, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_H1_T0_OUT_H[0], 1, H1_T0_OUT_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBF, addr_H1_T0_OUT_L[0], 1, H1_T0_OUT_L, 1, I2C_TIMEOUT);

		H0_rH[0]	= H0_rH_x2[0]>>1;
		H1_rH[0]	= H1_rH_x2[0]>>1;

		H0[0]		= (H0_T0_OUT_H[0]<<8) + H0_T0_OUT_L[0];
		H1[0]		= (H1_T0_OUT_H[0]<<8) + H1_T0_OUT_L[0];
		H_OUT[0]	= (HUM_H[0]<<8) + HUM_L[0];

		//CALCUL DE L'HUMIDITE
		hum16[0] = ((int16_t)(H1_rH[0]-H0_rH[0]))*((int16_t)(H_OUT[0]-H0[0]))/((int16_t)(H1[0]-H0[0]))+(int16_t)(H0_rH[0]);
		printf("Humidity :%d\r\n", hum16[0]);
}
//PRESSURE
void pressure_IKSO1A2()
{
	    HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_REF_P_XL[0], 1, REF_P_XL, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_REF_P_L[0], 1, REF_P_L, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_REF_P_H[0], 1, REF_P_H, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_PRES_OUT_XL[0], 1, PRES_OUT_XL, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_PRES_OUT_L[0], 1, PRES_OUT_L, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_PRES_OUT_H[0], 1, PRES_OUT_H, 1, I2C_TIMEOUT);

		//CONCATENATION

		PRES_x4096[0]	= (PRES_OUT_H[0]<<16) + (PRES_OUT_L[0]<<8) + PRES_OUT_XL[0];
		REF_P_x4096[0]	= (REF_P_H[0]<<16) + (REF_P_L[0]<<8) + REF_P_XL[0];

		//CALCUL DE LA PRESSION
		pres32[0] = (REF_P_x4096[0])>>12;
		printf("Pressure :%"PRIu32"\r\n", pres32[0]);
}
//MAGNETO
void INIT_MAGNETO_LSM6DSL()
{
	HAL_I2C_Mem_Write(&hi2c1,0x3C, addr_CFG_REG_A_M[0], 1, CFG_REG_A_M, 1, I2C_TIMEOUT);
}
void MAGNETO_LSM6DSL()
{
		HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTX_L_REG_M[0], 1, OUTX_L_REG_M, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTX_H_REG_M[0], 1, OUTX_H_REG_M, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTY_L_REG_M[0], 1, OUTY_L_REG_M, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTY_H_REG_M[0], 1, OUTY_H_REG_M, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTZ_L_REG_M[0], 1, OUTZ_L_REG_M, 1, I2C_TIMEOUT);
		HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTZ_H_REG_M[0], 1, OUTZ_H_REG_M, 1, I2C_TIMEOUT);

		//CONCATENATION
		magnX16[0] = (OUTX_H_REG_M[0]<<8) + OUTX_L_REG_M[0];
		magnY16[0] = (OUTY_H_REG_M[0]<<8) + OUTY_L_REG_M[0];
		magnZ16[0] = (OUTZ_H_REG_M[0]<<8) + OUTZ_L_REG_M[0];

		//GESTION VALEURS NEGATIVES
		magnX16[0]	= est_negatif(magnX16[0]);
		magnY16[0]	= est_negatif(magnY16[0]);
		magnZ16[0]	= est_negatif(magnZ16[0]);

		magn16[0]	= ((magnX16[0]^2) + (magnY16[0]^2) + (magnZ16[0]^2))^(1/2);
}
//ACCELEROMETER
void INIT_ACCELERO_LSM6DSL()
{
	HAL_I2C_Mem_Write(&hi2c1,0x32, addr_CTRL_REG1_ACCEL[0], 1, CTRL_REG1_ACCEL, 1, I2C_TIMEOUT);
}
void ACCELERO_LSM6DSL()
{
	HAL_I2C_Mem_Read(&hi2c1,0x33, addr_X_LI_L[0], 1, X_LI_L, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x33, addr_X_LI_H[0], 1, X_LI_H, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x33, addr_Y_LI_L[0], 1, Y_LI_L, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x33, addr_Y_LI_H[0], 1, Y_LI_H, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x33, addr_Z_LI_L[0], 1, Z_LI_L, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x33, addr_Z_LI_H[0], 1, Z_LI_H, 1, I2C_TIMEOUT);

	accelX16[0] = (X_LI_H[0]<<8) + X_LI_L[0];
	accelY16[0] = (Y_LI_H[0]<<8) + Y_LI_L[0];
	accelZ16[0] = (Z_LI_H[0]<<8) + Z_LI_L[0];

	accelX16[0] = est_negatif(accelX16[0]);
	accelY16[0] = est_negatif(accelY16[0]);
	accelZ16[0] = est_negatif(accelZ16[0]);
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
