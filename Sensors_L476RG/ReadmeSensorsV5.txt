ReadmeSENSORS_V4

Connection with nucleo L476RG

  *  	MQ-135 gaz sensor connection :
  *     Put PA4 <-> D0 //digital
  * 	GND
  * 	3,3V
  *
  * 	Utilisation avec putty; vérifier COM mettre sérial num COM et un nom au pif dans saved sessions, flow control:non; aller dans serial mettre bon COM et bon baud revenir init save
  *	    printf
  *
  *	    add IKSO1A2 : Temperature and Humidity & in V5 pressure

  *	    Affiche toutes les 5 secs :
  *	    Sensors
  *	    	----------------------------
  *		SENSOR Gaz : 3570
  *		Temperature :241
  *		Humidity :47
  *		Pressure :1028
  *		----------------------------
  *
  Functions: 
void READ_SENSOR_MQ135(void);// Read sensor gaz
void ADC_Init(void); //Read ADC
void Who_am_i(void); //Send who_am_i data; must send 188 just to check
void Register(void); //Update data for temperature and humidity
void temperature_IKSO1A2(void);// to have temperature data  with device IKSO1A2
void humidity_IKSO1A2(void); //to have humidity data with device IKSO1A2
int _write(int file,char *ptr, int len); //function to printf with putty
