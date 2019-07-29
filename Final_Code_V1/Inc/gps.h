/*
 * gps.h
 *
 *  Created on: 8 mars 2019
 *      Author: Maxime
 */

#ifndef GPS_H_
#define GPS_H_

#include "stm32l4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"








/* Convert a uint32 into a uint8 array in hexadecimal format (used in the LORA_AT_SEND function to build the payload) */


uint8_t * CONV_CHAR32(uint32_t val, uint8_t * tab){

/* itoa converts a number to a string with the radix dpecified */
	itoa(val,tab,16);
	return tab;
}






/* Get GPS position by extracting the GPGGA frame, with a timeout condition to exit the function if it cannot reach a good value */

uint32_t* GPS_GETPOS(uint32_t* tab){

	/* Initialize a timeout and a timeout counter to exit the function if the GPS does not have any GPS signal */
int GPS_TIMEOUT=500;
int cpt_timeout=0;

/* Buffers to catch the GPGGA frame */

uint8_t RX_BUFF_SIZE=200;
uint8_t in_get1[RX_BUFF_SIZE];
uint8_t in_get2[RX_BUFF_SIZE];
uint8_t in_get3[RX_BUFF_SIZE];
uint8_t in_get4[RX_BUFF_SIZE];
uint8_t in_get5[RX_BUFF_SIZE];
uint8_t in_get6[RX_BUFF_SIZE];

/* pointers to catch the GPS frame and its end */

char * p=0x0;
char *q=0x0;


uint8_t DATA_VALIDE[2];




uint8_t k=0;

	do	{

/* Init and Deinit UART to make it work fine, if we don't do it the UART never change state */
		HAL_UART_DeInit(&huart1);
		HAL_UART_Init(&huart1);

/* Launch an acquisition of UART, as we can not get a buffer exceeding 200 bytes, we do 6 acquisition of 200 bytes */

HAL_StatusTypeDef status = 	HAL_UART_Receive(&huart1, (uint8_t *)in_get1, RX_BUFF_SIZE, 100);

						    HAL_UART_Receive(&huart1, (uint8_t *)in_get2, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get3, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get4, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get5, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get6, RX_BUFF_SIZE, 100);



/* Use of the function strstr from stdlib.h to find the first occurence of $GPGGA in the UART buffers */

							p=strstr((char*)in_get1,"$GPGGA");

/* Use of p=0x0, if the value of p is unchaged, test if we have the $GPGGA frame in the next buffer */

							if (p==0x0){

								p=strstr((char*)in_get2,"$GPGGA");
							}



							if (p==0x0){

										p=strstr((char*)in_get3,"$GPGGA");
														}


							if (p==0x0){

										p=strstr((char*)in_get4,"$GPGGA");
														}



							if (p==0x0){

										p=strstr((char*)in_get5,"$GPGGA");
														}


							if (p==0x0){

										p=strstr((char*)in_get6,"$GPGGA");
							}

/* Init and deinit UART to mae it work fine */

							HAL_UART_DeInit(&huart1);
							HAL_UART_Init(&huart1);
							MX_USART1_UART_Init();

/* Initialize a pointer 'q' to the end of the frame collected */

							 q=strstr(p,"\r\n");



/* The counter is incremented as we have done one iteration, usually we do 3 iterations, 5 maximum when tested */
cpt_timeout++;

/* Condition of end : if the frame is too long or too short or not found AND the timeout is not reached, stay in the "while"
 *
 * It may happen that the GPS frame $GPGGA is found, but it can be at the end of a buffer, or the frame can be incomplete.
 * The frame is around 70 characters long, so we assure that the frame is correct with the <50 and >100 condition.
 *
 */

}while(((q-p>100)||(q-p==0)||(q==0x0)||(q-p<50))&&(cpt_timeout<GPS_TIMEOUT));


	/* if timeout has been reached, get out of the function and give false position known by the user */


	if (cpt_timeout>=GPS_TIMEOUT){
		tab[0]=43;
		tab[1]=5;
		tab[2]=1;
		tab[3]=0;
		return tab;
	}






//******************* Conversion from minutes and second degrees to decimal degrees *********************

	/* In the following steps, we take in account the fact that we will send data through LoRa with Cayenne LPP payload format
	 * To achieve that we shall not take every digit for each position as it would do a bad conversion later
	 */



	/* Latitude acquisition and conversion */
/* Skip the fields that we do not want */
HAL_Delay(10);
p=strchr(p,',');
p++;
p=strchr(p,',');
p++;

uint8_t LAT_H[3],LAT_M[3],LAT_L[4];
LAT_H[0]=*p;
p++;
LAT_H[1]=*p;
p++;
LAT_H[2]='\0';

LAT_M[0]=*p;
p++;
LAT_M[1]=*p;
p++;
LAT_M[2]='\0';



p++;
LAT_L[0]=*p;
p++;
LAT_L[1]=*p;
p++;
LAT_L[2]=*p;
p++;
LAT_L[3]='\0';

uint32_t LAT=0;
uint32_t nb_LAT_H=0;
uint32_t nb_LAT_M=0;
uint32_t nb_LAT_L=0;

/* Atoi converts a string to a number */

nb_LAT_H=atoi(LAT_H);
nb_LAT_M=atoi(LAT_M);
nb_LAT_L=atoi(LAT_L);

/* We compute the data so we have a decimal degree position */

LAT=nb_LAT_H*10000+(uint32_t)((nb_LAT_M*10000)/60)+(uint32_t)((nb_LAT_L*10000)/36000);



/* Longitude acquisition and conversion */

/* Here we repeat the work done on the latitude */


HAL_Delay(10);
p=strchr(p,',');
p++;
p=strchr(p,',');
p++;
p++;

uint8_t LON_H[3],LON_M[3],LON_L[4];
LON_H[0]=*p;
p++;
LON_H[1]=*p;
p++;
LON_H[2]='\0';

LON_M[0]=*p;
p++;
LON_M[1]=*p;
p++;
LON_M[2]='\0';



p++;
LON_L[0]=*p;
p++;
LON_L[1]=*p;
p++;
LON_L[2]=*p;
p++;
LON_L[3]='\0';

int32_t LON=0;
uint32_t nb_LON_H=0;
uint32_t nb_LON_M=0;
uint32_t nb_LON_L=0;

nb_LON_H=atoi(LON_H);
nb_LON_M=atoi(LON_M);
nb_LON_L=atoi(LON_L);

LON=nb_LON_H*10000+(uint32_t)((nb_LON_M*10000)/60)+(uint32_t)((nb_LON_L*10000)/36000);

HAL_Delay(10);



p=strchr(p,',');
p++;
/* Check the orientation, if it is East the longitude is positive, else it is negative */
uint8_t orientation[1]={*p};
if (orientation[0]=='O'){
	LON=-LON;
}

/* Check if the data is valid, 0 : not valid, 1 valid with GPS fix, 2 valid with DGPS fix */
p=strchr(p,',');
p++;
DATA_VALIDE[0]=*p;
DATA_VALIDE[1]='\0';

p=strchr(p,',');
p++;
p=strchr(p,',');
p++;
p=strchr(p,',');
p++;

/* Get the altitude, if the altitude is under 0 we get the absolute value of altitude
 * We only get absolute altitude, as the balloon will go up only
 *  */

char ALT[15];
while (*p!='.'){
	if (*p=='-'){
		p++;
	};
	ALT[k]=*p;
	p++;
	k++;
	}

ALT[k]='\0';
HAL_Delay(10);



/* return the array with the data, we multiply the altitude by 100 to fot the Cayenne LPP format */

tab[0]=LAT;
tab[1]=LON;
tab[2]=atoi(ALT)*100;
tab[3]=atoi(DATA_VALIDE);




p=0x0;
return tab;

}








#endif /* GPS_H_ */
