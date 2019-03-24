Low Power Mode 
 Put this document your library L4 at this link below
\en.stm32cubel4\STM32Cube_FW_L4_V1.13.0\Projects\NUCLEO-L476RG\Examples\RTC
 Open with your favourite IDE
 Compile the project and Run it (be careful, RUN is different of the debug mode, so click on RUN MODE)
 Press the Reset button and
 You can see the functionality of the standby mode : the LED is lighting.
 The code function is simple, we put the STM32 in "standby mode" and we wake up it on the RTC Alarm.

Read the code I put comments. 


Regarding the final code you have to add :
- static void RTC_AlarmConfig(void); 
- configure the RTC peripheral
- configure the clock 