#include "flash.h"
#include "stdio.h"
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_16   /* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_255 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PAGEError = 0;
__IO uint32_t data32 = 0 ;
uint32_t	 MemoryProgramStatus = 0;
/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;



//void write_flash(uint32_t addr, uint8_t data){
//	
//    HAL_FLASH_Unlock();
//	
//    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR);
//    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data);
//	
//    HAL_FLASH_Lock();
//}

//uint8_t read_flash (uint32_t Addr){
//	
//	return *(__IO uint64_t*)Addr;

//}

uint32_t algo_flash_test(uint32_t	Address, uint32_t Address_end, uint32_t  data_verif, uint64_t data){
	
	uint32_t write_error = 0x00;
	uint32_t read_error = 0x00;
	read_error = ReadFlash(Address, data_verif);
	write_error = WriteFlash(Address, data, Address_end);

	
	return read_error;
}

void InitFlash(uint32_t Addres_start, uint32_t Addres_end){
	 /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
  /* Get the 1st page to erase */
  FirstPage = GetPage(Addres_start); //start adr
  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(Addres_end) - FirstPage + 1;
  /* Get the bank */
  BankNumber = GetBank(Addres_start);
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;
}











uint64_t EraseFlash(uint32_t Addres_start, uint32_t Addres_end){
	InitFlash(Addres_start,Addres_end);
	
	uint64_t error_flash_erase = 0x00;

	HAL_FLASH_Unlock();

  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PAGEError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
			error_flash_erase++;

  }
		/* Lock the Flash to disable the flash control register access (recommended
		to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();
		
	return error_flash_erase;
}

uint32_t WriteFlash(uint32_t Address, uint64_t data, uint32_t Address_end){
	
  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	InitFlash(Address,Address_end);
//  Address = FLASH_USER_START_ADDR;
	uint32_t error_flash_write = 0x00;
	HAL_FLASH_Unlock();

  while (Address < Address_end)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data) == HAL_OK)
    {
      Address = Address + 8;
    }
   else
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
			error_flash_write++;
    }
  }
	/* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	
	return error_flash_write;
}
	
uint32_t ReadFlash( uint32_t	Address, uint32_t  data_verif){
  /* Check if the programmed data is OK
      MemoryProgramStatus = 0: data programmed correctly
      MemoryProgramStatus != 0: number of words not programmed correctly ******/
//  Address = FLASH_USER_START_ADDR;
	
  MemoryProgramStatus = 0x0;
	uint64_t error_flash_read = 0x00;

  while (Address < ADDR_FLASH_PAGE_255)
  {
    data32 = *(__IO uint32_t *)Address;

    if (data32 != data_verif)
    {
      MemoryProgramStatus++;
    }
    Address = Address + 4;
  }

  /*Check if there is an issue to program data*/
  if (MemoryProgramStatus == 0)
  {
    /* No error detected. Switch on LED2*/
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
		
  }
  else
  {
    /* Error detected.*/
		error_flash_read++;
  }
	return MemoryProgramStatus; 
}


/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}
