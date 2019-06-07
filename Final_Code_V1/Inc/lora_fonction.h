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
#include "gps.h"







#define LORAWAN_APP_DATA_BUFF_SIZE                           64





// Converts a uint_16 value into a uint_8 * value with hexadecimal format
uint8_t * CONV_CHAR(uint16_t val, uint8_t * tab){

	itoa(val,tab,16);
	return tab;
}


/* Function used to configure the keys and set the lora */


uint8_t LORA_AT_SET(uint8_t* comm){
	uint8_t buffer[128] = {'\0'};
	uint8_t recep_buff[128]={'\0'};
int i=5;
	sprintf(buffer,"%s\r\n",comm);
	HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 100);
	HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, recep_buff,40,10);

	if (recep_buff[0]=='O'){
		i++;
		return 1;
	}

	else{
		i--;
		return 0;
	}
}


/* Function used to get lora configurtion state and keys such as DevEUI */

void LORA_AT_GET(uint8_t* comm,uint8_t RX_SIZE){
	uint8_t buffer[128] = {'\0'};
	uint8_t recep_buff[128]={'\0'};
int i=5;
	sprintf(buffer,"%s\r\n",comm);
	HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 1000);
	HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, recep_buff,40,1000);

	if (recep_buff[0]=='O'){
		i++;
		return 1;
	}

	else{
		i--;
		return 0;
	}
	}




/* Function used to reset the lora configuration */

void LORA_AT_ATZ(void){

uint8_t  out[]={"ATZ\r\n"};
uint8_t RX_BUFF_SIZE=2;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}





/* Function used to check the joined status */

void LORA_AT_JSDA(void){

uint8_t  out[]={"AT+JSTA\r\n"};
uint8_t RX_BUFF_SIZE=5;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);
}


/* Function used to define the lora join mode, OTAA =1, ABP=0 */


uint8_t LORA_AT_JOIN_SET(uint8_t ota_mode){
	uint8_t buffer[32] = {'\0'};
	sprintf(buffer,"AT+JOIN=%d\r\n",ota_mode != 0);
	HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 1000);
	HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, buffer,20,100);
	return 1;
}


/* Function used to send data */

void LORA_AT_SEND( const uint8_t* LON,const uint8_t* LAT,const uint8_t* ALT,const uint8_t* DATA_TEMP,const uint8_t* DATA_HUM,const uint8_t* DATA_PRES){

uint8_t buffer[128] = {'\0'};
uint8_t comm[]={"AT+SEND=02,"};
uint8_t COMMAND_SIZE=sizeof(comm);
uint8_t recep_buff[128]={'\0'};
char GPS_MASK[5]={"0288"},longi[8]={"\0"},lati[8]={"\0"},alti[8]={"\0"};

char CapTemp[5]={"67"},CapHumi[]={"68"},Hum[3]={"\0"},Temp[5]={"\0"},CapPres[]={"73"},Pres[5]={"\0"};


	uint8_t siz_lon=strlen(LON);
	uint8_t siz_lat=strlen(LAT);
	uint8_t siz_alt=strlen(ALT);


					for(uint8_t i=0;i<6-siz_lon;i++){
						strcat(longi,"0");
					}
					strcat(longi,LON);



					for(uint8_t i=0;i<6-siz_lat;i++){
						strcat(lati,"0");
					}
					strcat(lati,LAT);



					for(uint8_t i=0;i<6-siz_alt;i++){
						strcat(alti,"0");
					}
					strcat(alti,ALT);



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




							     		if (strlen(DATA_PRES)==1){
							     			strcat(Pres, "000");
							     		}
							    	    if (strlen(DATA_PRES)==2){
							     	     	 strcat(Pres, "00");
							     	    }
							     		if (strlen(DATA_PRES)==3){
							       			strcat(Pres, "0");
							     		}

							     		strcat(Pres, DATA_PRES);



		    sprintf(buffer,"AT+SEND=2,%s%s%s%s02%s%s02%s%s02%s%s,0\r\n",GPS_MASK,longi,lati,alti,CapTemp,Temp,CapHumi,Hum,CapPres,Pres);
//02%s%s02%s%s
		     		HAL_UART_Transmit(&hlpuart1, buffer, strlen(buffer), 100);
		     		HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, recep_buff,20,100);


HAL_Delay(100);

}

#endif /* LORA_FONCTIONS_H_ */
