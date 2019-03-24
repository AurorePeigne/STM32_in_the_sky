/*
 * lora_fonctions.h
 *
 *  Created on: 21 févr. 2019
 *      Author: Max
 */

#ifndef LORA_FONCTION_H_
#define LORA_FONCTION_H_

#include "stm32l4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "I2C_stack.h"


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
uint8_t * CONV_CHAR(uint16_t val, uint8_t * tab){


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




/* ************************************************************************
 * ************************************************************************
 * 							FONCTIONS LORA SET
 * ************************************************************************
 * ************************************************************************
 */


uint8_t LORA_AT_SET(uint8_t* comm){
	uint8_t buffer[128] = {'\0'};
	uint8_t recep_buff[128]={'\0'};
int i=5;
	sprintf(buffer,"%s\r\n",comm);
	HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 1000);
	HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, recep_buff,4,1000);

	if (recep_buff[0]=='O'){
		i++;
		return 1;
	}

	else{
		i--;
		return 0;
	}
}



void LORA_AT_GET(uint8_t* comm,uint8_t RX_SIZE){
		uint8_t buffer[128] = {'\0'};
		uint8_t recep_buff[128]={'\0'};

		sprintf(buffer,"%s\r\n",comm);
		HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 1000);
		HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, recep_buff,RX_SIZE,1000);

		HAL_Delay(100);
	}


//
///* ************************************************************************
// * ************************************************************************
// * 							FONCTIONS LORA RESET
// * ************************************************************************
// * ************************************************************************
// */
//




void LORA_AT_ATZ(void){
/* Attention ceci c'est pour reset le tranceiver, je te laisse lire la doc ! */
uint8_t  out[]={"ATZ\r\n"};
uint8_t RX_BUFF_SIZE=2;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}







void LORA_AT_JSDA(void){
/* On check le Join Status en OTAA, pour du debug */
uint8_t  out[]={"AT+JSTA\r\n"}; //joined status
uint8_t RX_BUFF_SIZE=5;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}


/* ************************************************************************
 * ************************************************************************
 * 						FONCTIONS LORA COMMUNICATIONS
 * ************************************************************************
 * ************************************************************************
 */





uint8_t LORA_AT_JOIN_SET(uint8_t ota_mode){
	uint8_t buffer[32] = {'\0'};
	sprintf(buffer,"AT+JOIN=%d\r\n",ota_mode != 0);
	HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 1000);
	HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, buffer,20,15000);
	return 1;
}

void LORA_AT_SEND( const uint8_t* DATA_TEMP,const uint8_t* DATA_HUM,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){

uint8_t buffer[128] = {'\0'};
uint8_t comm[]={"AT+SEND=02,"};
uint8_t COMMAND_SIZE=sizeof(comm);
uint8_t recep_buff[128]={'\0'};
char CapTemp[5]={"0267"},CapHumi[]={"68"},Hum[3]={"\0"},Temp[5]={"\0"};

					if (strlen(DATA_TEMP)==1){
		     	    	 strcat(Temp, "000");
		     	     }
		     	     if (strlen(DATA_TEMP)==2){
		     	     		 strcat(Temp, "00");
		     	     }
		     		if (strlen(DATA_TEMP)==3){
		     				strcat(Temp, "0");
		     		}
		     		  strcat(Temp, DATA_TEMP);

		     		 if (strlen(DATA_HUM)==1){
		     			     strcat(Hum, "0");
		     		 }

		     		 strcat(Hum, DATA_HUM);
		    // sprintf(buffer,"AT+SEND=2,CAFECAFECAFECAFE,1\r\n");
		    sprintf(buffer,"AT+SEND=2,%s%s02%s%s,0\r\n",CapTemp,Temp,CapHumi,Hum);

		     		HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 1000);
		     		HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, recep_buff,8,5000);

 HAL_Delay(100);

}

#endif /* LORA_FONCTIONS_H_ */
