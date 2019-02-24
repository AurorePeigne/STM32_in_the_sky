Projet g�n�r� a partir de cube avec la board STM32L476RG

LPUART asynchronous

V1 - 

	Utilisation du LPUART pour envoyer des informations au shield LoRa I-NUCLEO-LRWAN1
	Prototype pour cr�er des fonctions servant a communiquer en LoRa
	
Brancher le LPUART_Tx du Shield au pin A5
Brancher le LPUART_Rx du Shield au pin A4

Ne pas oublier la masse
Alim 3.3V


Il faut remplacer le main du projet par les main de version plus r�cente
Il faut ajouter le fichier lora_fonctions.h au projet, en tant que header file
clic droit sur le projet->New header file-> Ctrl+C/V

Le fichier lora_fonctions.h contient la stack LoRa pour parler en uart avec le tranceiver
Il est grandement conseill� de jeter un oeil sur les AT command en m�me temps
que vous travaillez sur ce fichier
(AT_command joint dans le dossier)




Il y a donc des fonctions SET pour fixer la valeur de la variable
Des fonctions GET pour v�rifier les variables pour faire du debug

Des fonctions autres pour reset le lora ou avoir des status d'�tat
que l'on ne peut pas "set"

Pour les fonctions, le principe est simple: 

SET -> 3 ARGUMENTS -> la valeur de la variable a set, la taille de cette valeur, la taille du tableau de retour
�a peut paraitre d�bile mais j'ai pas r�ussi a faire marcher sans �a

GET -> fonction void, tout est initialis� dans la fonction

Si vous voulez lancer et voir ce qui se passe, il faut ajouter des breakpoints a la fin de la
fonction que vous voulez �tudier *EXEMPLE ci apres

Il faut ajouter dans les Expressions (Atollic) out et in_get ou in_set
(�a d�pend de la fonction, si c'est set ou get)


Les SET doivent r�pondre "OK\r\n" (expression de in_set)

/************************  EXEMPLE ************************/


void LORA_AT_EUI_GET(void){
//Oblig� de mettre la taille de retour attendue pour la commande
//uint8_t RX_BUFF_SIZE=53;
//On entre directement ici la commande et initialise le tableau de retour
uint8_t  out[]={"AT+EUI\r\n"};
uint8_t RX_BUFF_SIZE=29;
uint8_t in_get[RX_BUFF_SIZE];
    	while(HAL_UART_Transmit(&hlpuart1, (uint8_t *)out, sizeof(out), HAL_TIMEOUT) != HAL_OK);
     	UART_EndTxTransfer(&hlpuart1);

      	while(HAL_UART_Receive(&hlpuart1, (uint8_t *)in_get, RX_BUFF_SIZE, 100) != HAL_OK);
    	UART_EndRxTransfer(&hlpuart1);
    	HAL_Delay(100);

/*  BREAKPOINT ICI  */



}
