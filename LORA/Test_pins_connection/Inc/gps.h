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
/* Attention ceci c'est pour reset le tranceiver, je te laisse lire la doc ! */
//uint8_t  out[]={"$GPGLL\r\n"};//{"AT\r\n"};
	uint8_t* buff[20]={'\0'};
uint8_t RX_BUFF_SIZE=200;
uint8_t in_get1[RX_BUFF_SIZE];
uint8_t in_get2[RX_BUFF_SIZE];
uint8_t in_get3[RX_BUFF_SIZE];
uint8_t in_get4[RX_BUFF_SIZE];
uint8_t in_get5[RX_BUFF_SIZE];
uint8_t in_get6[RX_BUFF_SIZE];
  //HAL_UART_Transmit(&huart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT);
  	//UART_EndTxTransfer(&huart1);
//uint8_t chaine1[]="chaine";
//uint8_t chaine2[]="cha";
char * p=0x0;
uint8_t i=0;
//p=strstr((char*)chaine1,(char*)chaine2);

while ((p[45]!='1')&&(p[45]!='2')){
		while((p[74]!='\r')&&(p[75]!='\n')){



HAL_StatusTypeDef status = 	HAL_UART_Receive(&huart1, (uint8_t *)in_get1, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get2, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get3, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get4, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get5, RX_BUFF_SIZE, 100);

							HAL_UART_Receive(&huart1, (uint8_t *)in_get6, RX_BUFF_SIZE, 100);
//$GPGLL
							//$GPGGA




							p=strstr((char*)in_get1,"$GPGGA");
								i=2;
							if (p==0x0){
									//i=3;
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
									//i=6;
										p=strstr((char*)in_get5,"$GPGGA");
														}


							if (p==0x0){
									i=0;
										p=strstr((char*)in_get6,"$GPGGA");
							}
							char b = p[74];
							char a = p[75];
							//UART_EndRxTransfer(&huart1);
							 MX_USART1_UART_Init();
			}
}


char LONG[8];
char LAT[8];
char ALT[8];


char LONG16[20]={'\0'};
char LAT16[20]={'\0'};
char ALT16[20]={'\0'};

uint8_t x=p[45];


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





//uint32_t nombres[]={LONG,LAT,ALT};
CONV_CHAR32(atoi(LONG),LONG16);
CONV_CHAR32(atoi(LAT),LAT16);
CONV_CHAR32(atoi(ALT),ALT16);


tab[0]=atoi(LONG);
tab[1]=atoi(LAT);
tab[2]=atoi(ALT)*100;
char buff1[50];
char buff2[50];
char buff3[50];
char buff4[50];
lora_test(CONV_CHAR32(tab[0],buff1));
lora_test(CONV_CHAR32(tab[1],buff2));
lora_test(CONV_CHAR32(tab[2],buff3));
sprintf(buff4,"%s%s%s",buff1,buff2,buff3);
//strcpy(buff,buffer);
p=0x0;
return tab;
    //	HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, buffer,20,15000);
}










/* Tu fais des buffers de taille 20, tu comptes le nombre de virgules, entre chaque virgule tu remplis un buffer
 * avec la position, tu atoi les buffers, tu itoa les nombres, tu mets en forme cayenne lpp, tu sprintf le tout, bingo
 */



#endif /* GPS_H_ */
