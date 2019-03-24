GPS tracker with LoRa solution

To use the GPS shield on STM32 L476RG

You just plugged the STM32 with the shield : A green LED D3 on the right get on.

1-  Get software STM32CubeExpansion_GNSS1_V2.1.0
2- Open folder, then Projects -> Multi -> Applications -> GetPos -> Binary -> GetPos_UART-L476RG.bin
3- Once you are here, put the file in the NODE_L476RG peripheral (make it slide into it)

This allows the GPS shield to use UART communication, you can check if it worked thanks to the D2 red LED that toggles 
(not far from D2)

Then you juste have to run the code, that basically does a simple UART acquisition on the UART1.


