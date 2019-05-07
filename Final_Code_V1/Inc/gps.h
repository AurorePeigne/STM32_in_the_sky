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
//#include "I2C_stack.h"


/*************************************************************************************
 * ***********************************************************************************
 * ***************************A LIRE**************************************************
 * ***********************************************************************************
 * ***********************************************************************************
 *
 * Je commente bien qu'une set et une get, les autres du même type ont exactement le même principe
 * Je commente les premières de chaque type donc AK_SET et AK_GET
 *
 * Si tu débug cher ami, ne place pas de breakpoint entre les comm uart, je te le rapellerai dans une des fonctions !
 *
 * Il est conseillé d'avir la doc des AT command avec soi ! C'est plus pratique pour la suite
 */





/* ************************************************************************
 * ************************************************************************
 * 						FONCTIONS LPUART END TRANSFER
 * ************************************************************************
 * ************************************************************************
 */

#define LORAWAN_APP_DATA_BUFF_SIZE                           64



/* Très important !!!! Je convertis les valeurs renvoyées par les capteurs en chaine d'hexa pour les envoyer sous ce format */

uint8_t * CONV_CHAR32(uint32_t val, uint8_t * tab){


	itoa(val,tab,16);
	return tab;
}



static void UART_EndTxTransfer(UART_HandleTypeDef *huart)
{
#if defined(USART_CR1_FIFOEN)
  /* Disable TXEIE, TCIE, TXFT interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE));
  CLEAR_BIT(huart->Instance->CR3, (USART_CR3_TXFTIE));
#else
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
#endif

  /* At end of Tx process, restore huart->gState to Ready */
  huart->gState = HAL_UART_STATE_READY;
}


/**
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param huart UART handle.
  * @retval None
  */
static void UART_EndRxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
#if defined(USART_CR1_FIFOEN)
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));
  CLEAR_BIT(huart->Instance->CR3, (USART_CR3_EIE | USART_CR3_RXFTIE));
#else
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
#endif

  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;

  /* Reset RxIsr function pointer */
  huart->RxISR = NULL;
}


int lora_test(char* lon){



	char CapTemp[5]={"0288"},longi[8]={"\0"},Temp[5]={"\0"};


	uint8_t siz=strlen(lon);
						for(uint8_t i=0;i<6-siz;i++){
							strcat(longi,"0");
						}
strcat(longi,lon);

return 0;

}


uint32_t* GPS_GETPOS(uint32_t* tab){


uint8_t RX_BUFF_SIZE=200;
uint8_t in_get1[RX_BUFF_SIZE];
uint8_t in_get2[RX_BUFF_SIZE];
uint8_t in_get3[RX_BUFF_SIZE];
uint8_t in_get4[RX_BUFF_SIZE];
uint8_t in_get5[RX_BUFF_SIZE];
uint8_t in_get6[RX_BUFF_SIZE];

char * p=0x0;
char *q=0x0;
uint32_t taille=100;

uint8_t DATA_VALIDE[2];



char ALT[15];
uint8_t k=0;

	do	{



HAL_StatusTypeDef status = 	HAL_UART_Receive(&huart1, (uint8_t *)in_get1, RX_BUFF_SIZE, 100);

						    HAL_UART_Receive(&huart1, (uint8_t *)in_get2, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get3, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get4, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get5, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get6, RX_BUFF_SIZE, 100);





							p=strstr((char*)in_get1,"$GPGGA");

							if (p==0x0){

								p=strstr((char*)in_get2,"$GPGGA");
							}



							if (p==0x0){
										//i=4;
										p=strstr((char*)in_get3,"$GPGGA");
														}


							if (p==0x0){
										//i=5;
										p=strstr((char*)in_get4,"$GPGGA");
														}



							if (p==0x0){

										p=strstr((char*)in_get5,"$GPGGA");
														}


							if (p==0x0){

										p=strstr((char*)in_get6,"$GPGGA");
							}



							 MX_USART1_UART_Init();
	//		}

							 q=strstr(p,"\r\n");
taille=q-p;

}while((q-p>100)||(q-p==0)||(q==0x0));







//*****************************************  TEST CONVERSION DMS LAT->DD****************

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

nb_LAT_H=atoi(LAT_H);
nb_LAT_M=atoi(LAT_M);
nb_LAT_L=atoi(LAT_L);

LAT=nb_LAT_H*10000+(uint32_t)((nb_LAT_M*10000)/60)+(uint32_t)((nb_LAT_L*10000)/36000);



/*********************************************************************/
//*****************************************  TEST CONVERSION DMS LON->DD****************

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

uint32_t LON=0;
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


//*****************************************************************************/
/*
		// Longitude
		uint8_t k=0;
		for (uint8_t j=0;j<7;j++){
			if(p[18+j]!='.'){
				LONG[k]=p[18+j];
				k++;
			}
		}

LONG[7]='\0';






		//Latitude
		k=0;
			for (uint8_t j=0;j<8;j++){
				if(p[31+j]!='.'){
					LAT[k]=p[31+j];
					k++;
				}
			}

			LAT[7]='\0';






			//Altitude
			k=0;

				while(p[54+k]!='.'){
					ALT[k]=p[54+k];
					k++;
				}
ALT[k]='\0';

*/




tab[0]=/*atoi(LAT);*/LAT;
tab[1]=/*atoi(LONG);*/LON;
tab[2]=atoi(ALT)*100;

p=0x0;
return tab;
    //	HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, buffer,20,15000);
}









#endif /* GPS_H_ */
