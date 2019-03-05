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



void LORA_AT_AK_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
	/*
		 * Entete de fonction avec 3 variables : la data a transmettre (ici la AK)
		 * DATA_SIZE est la taille de la DATA que tu entres, c'est stupide JE SAIS mais j'ai pas réussi autrement et ça a marché comme ça
		 * le RX BUFF SIZE donne la taille du in set, une fois de plus stupide mais pareil je le fais pas marcher sans
		 *
		 * en général sur les set on te retourne OK\r\n donc tu mets 4 (sauf si tu vois des erreurs là tu mets plus pour debug)
		 *
		 */
uint8_t comm[]={"AT+AK="};
uint8_t COMMAND_SIZE=sizeof(comm);
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];
	     strcpy(src1,  DATA);
	     strcpy(src2,  "\r\n");
	     strcpy(out,comm );
	     /*
	   	      * On initalise des chaines de carac pour former la commande UART, on les concatène après
	   	      */
	     strcat(out, src1);
	     strcat(out, src2);


uint8_t in_set[RX_BUFF_SIZE];


    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);
     	/* PAS DE BREAKPOINT ICI, pourquoi ?
     	     	 * Tu vas predre les informations que te renvoie le tranceiver: le temps que tu prennes la pause t'auras loupé le message !
     	     	 * Si tu ne comptes pas recevoir tu peux en placer un sur l'instruction juste au dessus de ce commentaire, pour checker ton out[]
     	     	 */
      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);
//Breakpoint de debug ici: Tu peux check le in et le out ici ! Ils sont locaux a la fonction, tu ne les verras pas dans le main
}



void LORA_AT_APPEUI_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
uint8_t comm[]={"AT+APPEUI="};
uint8_t COMMAND_SIZE=sizeof(comm);
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];
	     strcpy(src1,  DATA);
	     strcpy(src2,  "\r\n");
	     strcpy(out,comm );
	     strcat(out, src1);
	     strcat(out, src2);
uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}

void LORA_AT_ASK_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
uint8_t comm[]={"AT+ASK="};
uint8_t COMMAND_SIZE=sizeof(comm);
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];
	     strcpy(src1,  DATA);
	     strcpy(src2,  "\r\n");
	     strcpy(out,comm );
	     strcat(out, src1);
	     strcat(out, src2);
uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}


void LORA_AT_NSK_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
uint8_t comm[]={"AT+NSK="};
uint8_t COMMAND_SIZE=sizeof(comm);
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];
	     strcpy(src1,  DATA);
	     strcpy(src2,"\r\n");
	     strcpy(out,comm );
	     strcat(out, src1);
	     strcat(out, src2);
uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}

void LORA_AT_CLASS_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
uint8_t comm[]={"AT+CLASS="};
uint8_t COMMAND_SIZE=sizeof(comm);
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];
	     strcpy(src1,  DATA);
	     strcpy(src2,  "\r\n");
	     strcpy(out,comm);
	     strcat(out, src1);
	     strcat(out, src2);
uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}

void LORA_AT_DC_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
uint8_t comm[]={"AT+DC="};
uint8_t COMMAND_SIZE=sizeof(comm);
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];
	     strcpy(src1,  DATA);
	     strcpy(src2,  "\r\n");
	     strcpy(out,comm );
	     strcat(out, src1);
	     strcat(out, src2);
uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}


void LORA_AT_DR_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){


uint8_t comm[]={"AT+DR="};
uint8_t COMMAND_SIZE=sizeof(comm);
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];
	     strcpy(src1,  DATA);
	     strcpy(src2,  "\r\n");
	     strcpy(out,comm);


	     strcat(out, src1);
	     strcat(out, src2);
uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}



/* ************************************************************************
 * ************************************************************************
 * 							FONCTIONS LORA GET
 * ************************************************************************
 * ************************************************************************
 */



void LORA_AT_AK_GET(void){

	/*
	 * FONCTION VOID, tranquille ici, besoin de rien, c'est quasiment que des fonctions de debug ou alors pour connaitre l'état du tranceiver
	 * pour lui refaire une beauté et set les paramètres que tu veux changer
	 * -> mettre des breakpoint pour VOIR la valeur attendue, tu peux modifier pour qu'elle renvoie la valeur, bonne chance !

Obligé de mettre la taille de retour attendue pour la commande

On entre directement ici la commande  out[] et initialise le tableau de retour avec le in get[___]


J'initialise directement la commande dans la fonction, je sais qu'on pourrait mieux faire mais je veux que ça marche
et c'est long a coder, mais l'idéal serait de faire une fonction get générique avec comme paramètre ce que l'on veut GET
*/


uint8_t  out[]={"AT+AK\r\n"};
//Ici j'ai testé tous les cas, le rx buff size est la taille attendue a chaque fois
uint8_t RX_BUFF_SIZE=53;
uint8_t in_get[RX_BUFF_SIZE];


    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);
/* PAS DE BREAKPOINT ICI, pourquoi ?
 * Tu vas predre les informations que te renvoie le tranceiver: le temps que tu prennes la pause t'auras loupé le message !
 *
 */
      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
    	//Breakpoint de debug ici: Tu peux check le in et le out ici ! Ils sont locaux a la fonction, tu ne les verras pas dans le main
}

void LORA_AT_APPEUI_GET(void){
//Obligé de mettre la taille de retour attendue pour la commande

//On entre directement ici la commande et initialise le tableau de retour
uint8_t  out[]={"AT+APPEUI\r\n"};
uint8_t RX_BUFF_SIZE=30;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	//HAL_Delay(100);
}

void LORA_AT_ASK_GET(void){
//Obligé de mettre la taille de retour attendue pour la commande
//uint8_t RX_BUFF_SIZE=53;
//On entre directement ici la commande et initialise le tableau de retour
uint8_t  out[]={"AT+ASK\r\n"};
uint8_t RX_BUFF_SIZE=53;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}


void LORA_AT_NSK_GET(void){

uint8_t  out[]={"AT+NSK\r\n"};
uint8_t RX_BUFF_SIZE=53;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}


void LORA_AT_CLASS_GET(void){

uint8_t  out[]={"AT+CLASS\r\n"};
uint8_t RX_BUFF_SIZE=5;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}

void LORA_AT_DR_GET(void){

uint8_t  out[]={"AT+DR\r\n"};
uint8_t RX_BUFF_SIZE=4;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}

void LORA_AT_DC_GET(void){

uint8_t  out[]={"AT+DC\r\n"};
uint8_t RX_BUFF_SIZE=5;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}

void LORA_AT_EUI_GET(void){

uint8_t  out[]={"AT+EUI\r\n"};
uint8_t RX_BUFF_SIZE=29;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}














/* ************************************************************************
 * ************************************************************************
 * 							FONCTIONS LORA RESET
 * ************************************************************************
 * ************************************************************************
 */





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





void LORA_AT_JOIN_SET( const char* DATA,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
uint8_t comm[]={"AT+JOIN="};
uint8_t COMMAND_SIZE=sizeof(comm);

/* Rien de nouveau ici si tu as suivi le reste des fonctions, du classique !
 * Ici on veut join le serveur via la geteway, lire la doc !
 */
	   char src1[DATA_SIZE],src2[2], out[COMMAND_SIZE+DATA_SIZE+2];

	     strcpy(src1,  DATA);
	     strcpy(src2,  "\r\n");
	     strcpy(out,comm );
	     strcat(out, src1);
	     strcat(out, src2);
uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 600000) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}


void LORA_AT_SEND( const uint16_t* DATA_TEMP,const uint8_t* DATA_HUM,uint8_t DATA_SIZE,uint8_t RX_BUFF_SIZE){
	/* ATTENTION, cette fonction est horrible, il se peut que le "out" finisse par buguer et ne plus afficher le "AT+SEND=0x,"
	 * Dans ce cas il faut copier coller la fctn send a partir du fichier .h ouvert dans un autre ide et virer cette fonction send
	 * A PART CA, ça donne un bon format : CAYENNE LPP icic pour la Température et l'Humidité
	 *Je rajoute des '0' parce qu'ils ne sont pas pris en compte par la fctn "itoa" qui transforme une valeur en chaine de carac base 16 ici
	 *Pour le format CAYENNE LPP il faut le bon nombre de bytes donc on doit les remplir avec les 0 ;)
	 */


uint8_t comm[]={"AT+SEND=02,"};
uint8_t COMMAND_SIZE=sizeof(comm);





	   char src1[1],src2[4],src3[2],src4[4], out[COMMAND_SIZE+DATA_SIZE+4];


/* On construit toutes les chaines de carac ici pour les remplir/concaténer par la suite */

	     strcpy(src2,DATA_TEMP);
	     strcpy(src4,DATA_HUM);
	     strcpy(src3,  ",0\r\n");
	     strcpy(out,comm );

	     /* TEMPERATURE SOUS FORMAT CAYENNE LPP    1 byte CHANNEL 1 byte 67=TEMPERATURE 2 byte DATA*/

	     	 	 	 strcat(out, "0267");
		     	     if (strlen(DATA_TEMP)==1){
		     	    	     strcat(out, "000");
		     	     }
		     	     if (strlen(DATA_TEMP)==2){
		     	     		 strcat(out, "00");
		     	     }
		     		if (strlen(DATA_TEMP)==3){
		     				strcat(out, "0");
		     		}
		     		/* On rajoute la température a la suite de la commande */
		     	     strcat(out, src2);

		     /* HUMIDITE SOUS FORMAT CAYENNE LPP    1 byte CHANNEL 1 byte 68=HUMIDITE 1 byte DATA */


		     	     strcat(out,"0268");
		     	   	     if (strlen(DATA_HUM)==1){
		     	   	    	     strcat(out, "0");
		     	   	     }

		     	   	     strcat(out, src4);

		     	   	 /* On rajoute l'humidité a la suite de la commande */

/* Ne pas oubler les caracteres de fin de chaine */
 strcat(out, src3);


uint8_t in_set[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out)-1, HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_set, RX_BUFF_SIZE, 600000) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);


HAL_Delay(100);

}

#endif /* LORA_FONCTIONS_H_ */
