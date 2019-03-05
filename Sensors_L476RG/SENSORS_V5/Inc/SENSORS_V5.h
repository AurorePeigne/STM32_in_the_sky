/*
 * SENSORS_V5.h
 *
 *  Created on: 5 mars 2019
 *      Author: pierre.perrin
 */

#ifndef SENSORS_V5_H_
#define SENSORS_V5_H_

#define I2C_TIMEOUT 1000

I2C_HandleTypeDef hi2c1;

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


//MAGNETO//////////////////////////////////////////////////////////////////////////////////////////////////////////

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


//ACCELERATION/////////////////////////////////////////////////////////////////////////////////////////////////////

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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t est_negatif(uint16_t valeur)
{
	if (valeur>=0x8000){
		valeur=~valeur+1;
	}
	return valeur;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TEMP_HUM_init(void)
{
	HAL_I2C_Mem_Write(&hi2c1,0xBE, addr_CTRL_REG1_TEMP[0], 1, CTRL_REG1_TEMP, 1, I2C_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PRES_init(void)
{
	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_CTRL_REG1_PRES[0], 1, CTRL_REG1_PRES, 1, I2C_TIMEOUT);

	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_CTRL_REG2_PRES[0], 1, CTRL_REG2_PRES, 1, I2C_TIMEOUT);

	HAL_I2C_Mem_Write(&hi2c1,0xBA, addr_INTERRUPT_CFG[0], 1, INTERRUPT_CFG, 1, I2C_TIMEOUT);
}


void MAGN_init(void)
{
	HAL_I2C_Mem_Write(&hi2c1,0x3C, addr_CFG_REG_A_M[0], 1, CFG_REG_A_M, 1, I2C_TIMEOUT);
}


void ACCEL_init(void)
{
	HAL_I2C_Mem_Write(&hi2c1,0x32, addr_CTRL_REG1_ACCEL[0], 1, CTRL_REG1_ACCEL, 1, I2C_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_TEMP(void)
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
	T0_OUT[0] 	= est_negatif(T0_OUT[0]);
	T1_OUT[0]	= est_negatif(T1_OUT[0]);
	T_OUT[0]	= est_negatif(T_OUT[0]);

	//CALCUL DES T0_degC ET T1_degC FINALES
	T0_msb[0]		= T0_T1_msb[0] & 0x3;
	T1_msb[0]		= (T0_T1_msb[0] & 0xC)>>2;
	T0_degC[0] 		= (T0_msb[0]<<8) + T0_degC_x8[0];
	T1_degC[0] 		= (T1_msb[0]<<8) + T1_degC_x8[0];
	T0_degC_DIV8[0]	= T0_degC[0]>>3;
	T1_degC_DIV8[0]	= T1_degC[0]>>3;

	//CALCUL DE LA TEMPERATURE
	temp16[0] = ((int16_t)(T_OUT[0]-T0_OUT[0]))*10*((int16_t)(T1_degC_DIV8[0]-T0_degC_DIV8[0]))/((int16_t)(T1_OUT[0]-T0_OUT[0]))+(int16_t)(T0_degC_DIV8[0])*10;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_PRES(void)
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
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_MAGN(void)
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

void get_ACCEL(void)
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


#endif /* SENSORS_V5_H_ */
