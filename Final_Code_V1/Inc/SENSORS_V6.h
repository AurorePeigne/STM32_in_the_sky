/*
 * SENSORS_V5.h
 *
 *  Created on: 5 mars 2019
 *      Author: pierre.perrin
 */

#ifndef SENSORS_V6_H_
#define SENSORS_V6_H_

#define I2C_TIMEOUT 1000	//timeout of I2C functions
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t) 2)

#include "string.h"

I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

//TEMPERATURE VARIABLES/////////////////////////////////////////////////////////////////////////////////////////////////////

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

uint16_t temp16u[1];
uint16_t temp16d[1];


//HUMIDITY VARIABLES////////////////////////////////////////////////////////////////////////////////////////////////////////

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


//PRESSURE VARIABLES/////////////////////////////////////////////////////////////////////////////////////////////////////////

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


//MAGNETO VARIABLES//////////////////////////////////////////////////////////////////////////////////////////////////////////

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


//ADC VARIABLES//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

uint16_t gas16;
uint16_t bat16;

//SD VARIABLES///////////////////////////////////////////////////////////////////////////////////////////////////////////////

FATFS myFATFS;
FIL myFILE;
UINT testByte;

//strings to write on the SD card

char temp_u_char[3];
char temp_d_char[2];
char hum_char[3];
char pres_char[5];
char magn_char[4];

char lon_char[20];
char lat_char[20];
char alt_char[20];

//char validechar[20];

char gas_char[20];
char bat_char[20];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//function to manage the negative values of the different data

uint16_t is_negative(uint16_t valeur)
{
	if (valeur>=0x8000)	//if the most significant bit is 1
	{
		valeur=~valeur+1;	//we take the complement
	}
	return valeur;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TEMP_HUM_init(void)
{
	//active power-down control
	HAL_I2C_Mem_Write(&hi2c1,0xBE, addr_CTRL_REG1_TEMP[0], 1, CTRL_REG1_TEMP, 1, I2C_TIMEOUT);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PRES_init(void)
{
	//disable one-shot mode
	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_CTRL_REG1_PRES[0], 1, CTRL_REG1_PRES, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_CTRL_REG2_PRES[0], 1, CTRL_REG2_PRES, 1, I2C_TIMEOUT);

	//enable autozero function and reset it
	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_INTERRUPT_CFG[0], 1, INTERRUPT_CFG, 1, I2C_TIMEOUT);
	HAL_Delay(10);
}


void MAGN_init(void)
{
	//configure output data rate
	HAL_I2C_Mem_Write(&hi2c1,0x3C, addr_CFG_REG_A_M[0], 1, CFG_REG_A_M, 1, I2C_TIMEOUT);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DMA_init()
{
	//active DMA to receive several data on the same ADC
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)aADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_TEMP(void)
{
	//read the temperature data using I2C
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

	//concatenation
	T0_OUT[0]	= (T0_H[0]<<8) + T0_L[0];
	T1_OUT[0]	= (T1_H[0]<<8) + T1_L[0];
	T_OUT[0]	= (T_OUT_H[0]<<8) + T_OUT_L[0];

	//negative values management
	T0_OUT[0] 	= is_negative(T0_OUT[0]);
	T1_OUT[0]	= is_negative(T1_OUT[0]);
	T_OUT[0]	= is_negative(T_OUT[0]);

	//calculation of T0_degC and T1_degC
	T0_msb[0]		= T0_T1_msb[0] & 0x3;
	T1_msb[0]		= (T0_T1_msb[0] & 0xC)>>2;
	T0_degC[0] 		= (T0_msb[0]<<8) + T0_degC_x8[0];
	T1_degC[0] 		= (T1_msb[0]<<8) + T1_degC_x8[0];
	T0_degC_DIV8[0]	= T0_degC[0]>>3;
	T1_degC_DIV8[0]	= T1_degC[0]>>3;

	//final calculation of the temperature
	temp16[0] = ((int16_t)(T_OUT[0]-T0_OUT[0]))*10*((int16_t)(T1_degC_DIV8[0]-T0_degC_DIV8[0]))/((int16_t)(T1_OUT[0]-T0_OUT[0]))+(int16_t)(T0_degC_DIV8[0])*10;

	//integer part of the temperature
	temp16u[0] = temp16[0]/10;
	//decimale part of the temperature
	temp16d[0] = temp16[0] - temp16u[0]*10;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_HUM(void)
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
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_PRES(void)
{
	//read the humidity data using I2C
	HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_REF_P_XL[0], 1, REF_P_XL, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_REF_P_L[0], 1, REF_P_L, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_REF_P_H[0], 1, REF_P_H, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_PRES_OUT_XL[0], 1, PRES_OUT_XL, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_PRES_OUT_L[0], 1, PRES_OUT_L, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0xBB, addr_PRES_OUT_H[0], 1, PRES_OUT_H, 1, I2C_TIMEOUT);

	//concatenation
	PRES_x4096[0]	= (PRES_OUT_H[0]<<16) + (PRES_OUT_L[0]<<8) + PRES_OUT_XL[0];
	REF_P_x4096[0]	= (REF_P_H[0]<<16) + (REF_P_L[0]<<8) + REF_P_XL[0];

	//pressure calculation
	pres32[0] = (REF_P_x4096[0])>>12;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_MAGN(void)
{
	//read the magnetometer data using I2C
	HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTX_L_REG_M[0], 1, OUTX_L_REG_M, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTX_H_REG_M[0], 1, OUTX_H_REG_M, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTY_L_REG_M[0], 1, OUTY_L_REG_M, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTY_H_REG_M[0], 1, OUTY_H_REG_M, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTZ_L_REG_M[0], 1, OUTZ_L_REG_M, 1, I2C_TIMEOUT);
	HAL_I2C_Mem_Read(&hi2c1,0x3D, addr_OUTZ_H_REG_M[0], 1, OUTZ_H_REG_M, 1, I2C_TIMEOUT);

	//concatenation
	magnX16[0] = (OUTX_H_REG_M[0]<<8) + OUTX_L_REG_M[0];
	magnY16[0] = (OUTY_H_REG_M[0]<<8) + OUTY_L_REG_M[0];
	magnZ16[0] = (OUTZ_H_REG_M[0]<<8) + OUTZ_L_REG_M[0];

	//negative values management
	magnX16[0]	= is_negative(magnX16[0]);
	magnY16[0]	= is_negative(magnY16[0]);
	magnZ16[0]	= is_negative(magnZ16[0]);

	//magnetometer calculation
	magn16[0]	= ((magnX16[0]^2) + (magnY16[0]^2) + (magnZ16[0]^2))^(1/2);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_ADC(void)
{
	if (HAL_ADC_Start(&hadc1)!= HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
	//get gas ADC value
	gas16 = aADCxConvertedValues[0];
	//get battery ADC value
	bat16 = aADCxConvertedValues[1];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void SD_SENSORS(uint32_t lon32, uint32_t lat32, uint32_t alt32)
{
	//conversion of temperature data into strings
	itoa(temp16u[0],temp_u_char,10);
	itoa(temp16d[0],temp_d_char,10);

	//conversion of the humidity data into a string
	itoa(hum16[0],hum_char,10);

	//conversion of the pressure data into a string
	itoa(pres32[0],pres_char,10);

	//conversion of the magnetometer data into a string
	itoa(magn16[0],magn_char,10);

	//conversion of ADC data into strings
	uint16_t bat_volt16 = (uint16_t) (bat16*100/4700);
	itoa(gas16,gas_char, 10);
	itoa(bat_volt16,bat_char, 10);

	//conversion of GPS data into strings
	uint32_t alti32 = alt32/100;
	itoa(lon32,lon_char,10);
	itoa(lat32,lat_char,10);
	itoa(alti32,alt_char,10);
//	itoa(valide,validechar,10);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	HAL_Delay(100);

	//if we connect successfully to the SDCard :
	if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
	{
		  //we change the state of the LED
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		  //define the file in witch we write the data
		  char myFileName[] = "RESULT.TXT\0";
		  //open the file to write in it without deleting what it already contains
		  f_open(&myFILE, myFileName, FA_WRITE | FA_OPEN_APPEND);

		  //write in the file
		  f_write(&myFILE, "T:\t", 3, &testByte);
		  f_write(&myFILE, temp_u_char, strlen(temp_u_char), &testByte);
		  f_write(&myFILE, ",", 1, &testByte);
		  f_write(&myFILE, temp_d_char, strlen(temp_d_char), &testByte);
		  f_write(&myFILE, "\t°C\t", 4, &testByte);

		  f_write(&myFILE, "P:\t", 3, &testByte);
		  f_write(&myFILE, pres_char, strlen(pres_char), &testByte);
		  f_write(&myFILE, "\thPa\t", 5, &testByte);

		  f_write(&myFILE, "H:\t", 3, &testByte);
		  f_write(&myFILE, hum_char, strlen(hum_char), &testByte);
		  f_write(&myFILE, "\t%\t", 3, &testByte);

		  f_write(&myFILE, "M:\t", 3, &testByte);
		  f_write(&myFILE, magn_char, strlen(magn_char), &testByte);
		  f_write(&myFILE, "\tmgauss\t", 8, &testByte);

		  f_write(&myFILE, "Lon:\t", 5, &testByte);
		  f_write(&myFILE, lon_char, strlen(lon_char), &testByte);
		  f_write(&myFILE, "\tm\t", 3, &testByte);
		  f_write(&myFILE, "Lat:\t", 5, &testByte);
		  f_write(&myFILE, lat_char, strlen(lat_char), &testByte);
		  f_write(&myFILE, "\tm\t", 3, &testByte);
		  f_write(&myFILE, "Alt:\t", 5, &testByte);
		  f_write(&myFILE, alt_char, strlen(alt_char), &testByte);
		  f_write(&myFILE, "\tm\t", 3, &testByte);

//		  f_write(&myFILE, "Val:\t", 5, &testByte);
//		  f_write(&myFILE, validechar, strlen(validechar), &testByte);
//		  f_write(&myFILE, "\t", 1, &testByte);

		  f_write(&myFILE, "G:\t", 3, &testByte);
		  f_write(&myFILE, gas_char, strlen(gas_char), &testByte);
		  f_write(&myFILE, "\tppm\t", 5, &testByte);

		  f_write(&myFILE, "Bat:\t", 5, &testByte);
		  f_write(&myFILE, bat_char, strlen(bat_char), &testByte);
		  f_write(&myFILE, "\t%\r\n", 4, &testByte);

		  //close the file
		  f_close(&myFILE);

		  HAL_Delay(900);

		  //we change the state of the LED
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}

}


#endif /* SENSORS_V6_H_ */
