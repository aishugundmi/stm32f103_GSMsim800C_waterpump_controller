#include "main.h"

static uint8_t eeprom_buff[1024];
static const uint32_t eeprom_base = 0x0801fc00; 

static uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t * DATA_32)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

        //write full page insted of partial... our data is exact copy of a page memory 
        //which is represented by eeprom_buff[1024];///
        //we do allmodifications in eeprom_buff and finall write it ocmpletely to last page of 
        //flash to backup it.
	int numberofwords = 1024/4;////(strlen(DATA_32)/4) + ((strlen(DATA_32) % 4) != 0);

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

          
           
           // trying to find  staring boundary of a page
           
           
           //in this example it will work without any change coze our address is already page aligned 
           //JUST KEEPIGN IT correct for future use... in case if we use different address 
          
	  uint32_t StartPage =  StartPageAddress -(StartPageAddress%FLASH_PAGE_SIZE) ;//(StartPageAddress - 0x08000000)/1024;  //GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofwords*4 - 1;
	  uint32_t EndPage = (EndPageAdress - ((EndPageAdress) % FLASH_PAGE_SIZE));// - 0x08000000)/1024;//GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.PageAddress = StartPage;
	   EraseInitStruct.NbPages     = ((EndPage - StartPage)/FLASH_PAGE_SIZE) + 1;

          
	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, DATA_32[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
}


static void Flash_Read_Data (uint32_t StartPageAddress, __IO uint32_t * DATA_32)
{

  for(int i = 0; i < 1024/4; i++) {
    *DATA_32 = *(__IO uint32_t *)StartPageAddress;    
    StartPageAddress += 4;
    DATA_32++;
  }
}

////////////////////////////////////////////////////////////////////////////////////////
//now after this all our operation will be on the 1kb ram buffer 
//then we will call this two functions to backup and take back 
void eeprom_write_to_flash(void)
{
  //write the 1kb eeprom ram buffer to flash memory
  Flash_Write_Data (eeprom_base, (void *)eeprom_buff);

}

void eeprom_read_from_flash(void)
{
  // write the 1 kb eeprom backup from flash to ram
  Flash_Read_Data (eeprom_base, (void *)eeprom_buff);
}

uint8_t *eeprom_get_buff_address(void)
{
  //this is for applicaiton layer to access the eeprom_buff address. 
  return eeprom_buff;
}