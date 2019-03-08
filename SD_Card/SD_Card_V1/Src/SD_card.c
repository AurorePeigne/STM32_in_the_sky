#include "SD_card.h"



	
uint8_t Store_data(uint8_t data){
	  
	FATFS myFATFS;
  FIL myFILE;
  UINT testByte;
	uint8_t err = 0;

if(f_mount(&myFATFS, SDPath, 1) == FR_OK)
  {
	  char myFileName[] = "DATA_ISEN_TL.TXT";

	  if(f_open(&myFILE, myFileName, (FA_WRITE | FA_CREATE_ALWAYS)) ==FR_OK){

			err = 0;
			
		char myData[255]; 
		sprintf(myData, "%d\n", data);
		 if(f_write(&myFILE, myData, sizeof(myData), &testByte) == FR_OK){

					err = 0;
		 }
		 else{
			 err = 1;
			}
		 f_close(&myFILE);
	  }
  }

  else{
			err = 1;
  }
	return err;
}

