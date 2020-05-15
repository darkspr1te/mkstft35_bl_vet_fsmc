#include "mkstft35.h"
#include "ff.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "flash_file.h"


extern unsigned int _isr_real;
extern const uint32_t *mcuFirstPageAddr;
#define FLASH_STEP 0x08
#define INCORRECT_SECTOR 0xFF



uint8_t buffer[FLASH_PAGE_SIZE];
uint32_t bufferLen = 0;

//static FIL fil;
static FILINFO info;

extern char SDPath[4];
extern FATFS SDFatFS;
extern FIL SDFile;
int key[] ={0xA3, 0xBD, 0xAD, 0x0D, 0x41, 0x11, 0xBB, 0x8D, 0xDC, 0x80, 0x2D, 0xD0, 0xD2, 0xC4, 0x9B, 0x1E, 0x26, 0xEB, 0xE3, 0x33, 0x4A, 0x15, 0xE4, 0x0A, 0xB3, 0xB1, 0x3C, 0x93, 0xBB, 0xAF, 0xF7, 0x3E};

HAL_StatusTypeDef crypt(uint32_t from, uint32_t to)
{

    //do {
    //FUN_08007b28(key,0xff,0x400);
    int counter, index,uVar7,uVar4;
   // uVar4 = FUN_08004980(puVar3,key,0x400,key + 0x440,local_24,local_2c);
  /*  key[0x414] = uVar4;
    if (*(uint *)(key + 0x44c) < 0x3c0) {
      if (key[0x416] == '\x01') {
        index = 0;
        uVar7 = 0;
        while (uVar7 < 0x16) {
          counter = 0;
          while (counter < 0x20) {
            (key + index)[0x140] = key[counter + 0x418] ^ (key + index)[0x140];
            index = index + 1;
            counter = counter + 1;
          }
          *(int *)(key + 0x44c) = *(int *)(key + 0x44c) + 1;
          uVar7 = uVar7 + 1;
        }
        key[0x416] = 0;
      }
      else {
        index = 0;
        uVar7 = 0;
        while (uVar7 < 0x20) {
          counter = 0;
          while (counter < 0x20) {
            key[index] = key[index] ^ key[counter + 0x418];
            index = index + 1;
            counter = counter + 1;
          }
          iVar8 = *(int *)(key + 0x44c);
          *(uint *)(key + 0x44c) = iVar8 + 1U;
          if (0x3bf < iVar8 + 1U) break;
          uVar7 = uVar7 + 1;
        }
      }
    }*/
}

FLASH_EraseInitTypeDef erase_flash;
HAL_StatusTypeDef erase(uint32_t from, uint32_t to)
{
	HAL_StatusTypeDef res = HAL_OK;



	for (uint32_t i = from; i <= to; i += FLASH_PAGE_SIZE)
	{
		FLASH_EraseInitTypeDef erase;

		//erase.TypeErase = FLASH_TYPEERASE_PAGES;
		//erase.Banks = FLASH_BANK_1;
	//	erase.PageAddress = i;
	//	erase.NbPages = 1;

	//	uint32_t error = 0;
	//	res = HAL_FLASHEx_Erase(&erase, &error);
#ifdef DEBUG		
		printf("erase page %x\n\r",i);
#endif
		if (res != HAL_OK) {
			return res;
		}
	}

//unit32_t address = 0x0800C000;
HAL_FLASH_Unlock();
__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
 //FLASH_Erase_Sector(x, VOLTAGE_RANGE_3);
for (int x=FLASH_SECTOR_3;x<FLASH_SECTOR_TOTAL;x++)
{
    //FLASH_Erase_Sector(x, VOLTAGE_RANGE_3);
}
HAL_FLASH_Lock();
//----------------------------write data  
/*
    uint8_t data = 'A';
    if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, start_address, data) != HAL_OK) 
    {
        HAL_FLASH_Lock();
        return;
    }
    HAL_FLASH_Lock();
	return res;
    */
   return res;
}

FRESULT readNextPage(uint8_t *target, uint32_t *read)
{
	uint16_t blocksCount = 16;
	uint16_t fileBlock = FLASH_PAGE_SIZE / blocksCount;

	*read = 0;
	UINT readNow = 0;
	FRESULT res = FR_OK;

	for (uint16_t i = 0; i<blocksCount; i++)
	{
		res = f_read(&SDFile, target, fileBlock, &readNow);

		target += readNow;
		*read += readNow;

		if (res != FR_OK || readNow != fileBlock)
			break;
	}
	return res;
}

HAL_StatusTypeDef flashWrite(uint32_t position, uint8_t *data, uint32_t size)
{	

  //for debug   printf("write page %x\n\r",position);
	
  HAL_StatusTypeDef res = HAL_OK;
	for (uint32_t i=0; i<size; i+=2)
	{
		
		res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, position + i, *(uint16_t*)&data[i]);
		if (res != HAL_OK)
        {
            printf("flash_file.c flash write error");
        	break;
        }
	}
	return res;
}

FlashResult flash(const char *fname)
{
	FRESULT res = f_stat(fname, &info);
 //   printf("About to flash system file size %dk\n\r",(info.fsize/1000) );
	if (res != FR_OK)
		return FLASH_FILE_NOT_EXISTS;

	// Checking file size
//	if (info.fsize > getMaxFlashImageSize())
//		return FLASH_FILE_TOO_BIG;

	res = f_open(&SDFile, fname, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK)
    {	  
	//  printf("Firmware Flash file open ok\n\r");
    }
	if (res != FR_OK)
	{
	
	  printf("Firmware Flash open file failed\n\r");
		return FLASH_RESULT_FILE_ERROR;
	}
	//uint32_t position = (uint32_t) mcuFirstPageAddr;
    uint32_t position = ADDR_FLASH_SECTOR_3;
	if (HAL_OK != HAL_FLASH_Unlock())
    {
        printf("flash unlock error\n\r");
    	return FLASH_RESULT_FLASH_ERROR;
    }
	

	if (HAL_OK !=  flash_erase(ADDR_FLASH_SECTOR_3,ADDR_FLASH_SECTOR_11))
	{
	
	printf("erase error\n\r");

		return FLASH_RESULT_FLASH_ERROR;
	}

	do {
		readNextPage(buffer, &bufferLen);

#ifdef DECRYPT
		if ((ADDR_FLASH_SECTOR_3-position)<=0x140 ||(ADDR_FLASH_SECTOR_3-position)<0x7940 )
    {
      if ((ADDR_FLASH_SECTOR_3-position)<=0x140)
      {
        printf("Using Stage one decrypt raw position %x\n\r",position);
        for (int i=0x140;i<bufferLen;i++)
        {
          buffer[i]=buffer[i] ^ key[i&31];
        }
      } else 
      if ((ADDR_FLASH_SECTOR_3-position)>=0x140||(ADDR_FLASH_SECTOR_3-position)<0x7940)
      {
        printf("Using Stage two decrypt raw position %x\n\r", position);
        for (int i=0x0;i<bufferLen;i++)
        {
          buffer[i]=buffer[i] ^ key[i&31];
        }
      }
      else 
      if ((ADDR_FLASH_SECTOR_3-position)>=0x3fff||(ADDR_FLASH_SECTOR_3-position)<0x7940)
      {
        printf("Using Stage three decrypt raw position %x\n\r", position);
        for (int i=0x0;i<bufferLen;i++)
        {
          buffer[i]=buffer[i] ^ key[i&31];
        }
      }
      

    }
    else
    {
      //printf("no decrypt\n\r");
      if ((ADDR_FLASH_SECTOR_3-position)<0x7940)
      {
        printf("Using Stage four decrypt raw position %x\n\r", position);
        for (int i=0x0;i<bufferLen;i++)
        {
          buffer[i]=buffer[i] ^ key[i&31];
        }
      }
    }
    
#endif
		if (HAL_OK != flashWrite(position, buffer, bufferLen))
			return FLASH_RESULT_FLASH_ERROR;

		position += bufferLen;
	} while (bufferLen != 0);

  printf("Finished flashing crypto at %x\n\r",position);
  printf("raw postition %x\n\r",(position-ADDR_FLASH_SECTOR_3));
	f_close(&SDFile);
	HAL_FLASH_Lock();
#ifdef DEBUG
	//  printf("flash write complete \r\n");
#endif
	return FR_OK;
}





///////////////////////////////////

static uint8_t clear_sector(uint32_t);
static uint32_t get_sector(uint32_t);

uint8_t flash_clear_sector(uint32_t addr) {
    uint32_t sector = get_sector(addr);
    if (sector == INCORRECT_SECTOR) return 0;
    return clear_sector(sector);
}
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_3   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_10   /* End @ of user Flash area */

#define DATA_32                 ((uint32_t)0x12345678)
uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
uint8_t flash_erase(uint32_t from_addr, uint32_t to_addr)
{
   // printf("1:FLASH_FILE.c flash erase from %x to %x\n\r",First,to_addr);
  // printf("flash_file.c - about to unlock\n\r");
  //  HAL_FLASH_Unlock();
  //  printf("flash_file.c - flash unlocked\n\r");
  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  //  printf("flash_file.c - flash flags complete\n\r");
  /* Get the 1st sector to erase */
   /* Get the 1st sector to erase */
  //FirstSector = GetSector(FLASH_USER_START_ADDR);
  FirstSector = GetSector(from_addr);
  /* Get the number of sector to erase from 1st sector*/
  NbOfSectors = GetSector(to_addr) - FirstSector + 1;
   // printf("2:FLASH_FILE.c flash erase from %d to %d\n\r",FirstSector,(FirstSector+NbOfSectors));
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;
  HAL_StatusTypeDef res = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
 // printf("Flash erase complete, code %d\n\r",res);
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
 // { 
 
    /* 
      Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
    */
      /*
        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
      */
  //   printf("An error occured in flash erase");
  //  Error_Handler();
 // HAL_FLASH_Lock();
    return 99;
 // }
return res;
}
uint8_t flash_clear(uint32_t from_addr, uint32_t to_addr) {
    uint8_t res;
    uint32_t first_sector = get_sector(from_addr);
    uint32_t last_sector = get_sector(to_addr);
   // printf("Flash erase from %x to %x\n\r",from_addr,to_addr);
   printf("We should not be here\r\n");
   return 0;
    if (first_sector == INCORRECT_SECTOR || last_sector == INCORRECT_SECTOR) return 0;

    while (first_sector != last_sector) {
        res = clear_sector(first_sector);
         printf("sector :%x\n\r",first_sector);
        if (res == 0) return 0;
        first_sector += FLASH_STEP;
    }

    return clear_sector(last_sector);
}

uint32_t flash_program_by_word(uint32_t addr, const uint32_t *data, uint32_t size) {
    HAL_StatusTypeDef res;
    HAL_FLASH_Unlock();
    //printf("flash_file.c program word %x\n\r",addr);
    for (uint32_t i = 0; i < size; ++i) {
        res = HAL_OK; 
   
        res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, *(uint16_t*)&data[i]);
      //  FLASH_Program_Word(addr, data[i]);
        if (res != HAL_OK) {
            HAL_FLASH_Lock();
            printf("flash_file.c - HAL write error");
            return 0;
        }
        addr += 4;
    }

   // HAL_FLASH_Lock();
    return 0;
}

uint32_t flash_program_by_byte(uint32_t addr, const uint8_t *data, uint32_t size) {
    HAL_StatusTypeDef res;
    HAL_FLASH_Unlock();
printf("flash_file.c program byte");
    for (uint32_t i = 0; i < size; ++i) {
        res = HAL_OK;
        res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, *(uint16_t*)&data[i]);
        //FLASH_Program_Byte(addr, data[i]);
        if (res != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
        ++addr;
    }

    HAL_FLASH_Lock();
    return addr;
}

static uint8_t clear_sector(uint32_t sector) {
    //unlock the FLASH control register access
  
    HAL_FLASH_Unlock();
    //Clear all FLASH pending flags
 //   __HAL_FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
 //                   FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    //VoltageRange_3 - operation will be done by word (32-bit)
    HAL_StatusTypeDef res = HAL_OK;
    FLASH_Erase_Sector(sector, VOLTAGE_RANGE_3);

    //lock the FLASH control register access
    HAL_FLASH_Lock();

    return (res == HAL_OK) ? 1 : 0;
}

static uint32_t get_sector(uint32_t addr) {
    if (!IS_FLASH_ADDRESS(addr)) return INCORRECT_SECTOR;

    if((addr >= FLASH_SECTOR_0_ADDR) && (addr < FLASH_SECTOR_1_ADDR))
        return FLASH_SECTOR_0;
    if((addr >= FLASH_SECTOR_1_ADDR) && (addr < FLASH_SECTOR_2_ADDR))
        return FLASH_SECTOR_1;
    if((addr >= FLASH_SECTOR_2_ADDR) && (addr < FLASH_SECTOR_3_ADDR))
        return FLASH_SECTOR_2;
    if((addr >= FLASH_SECTOR_3_ADDR) && (addr < FLASH_SECTOR_4_ADDR))
        return FLASH_SECTOR_3;
    if((addr >= FLASH_SECTOR_4_ADDR) && (addr < FLASH_SECTOR_5_ADDR))
        return FLASH_SECTOR_4;
    if((addr >= FLASH_SECTOR_5_ADDR) && (addr < FLASH_SECTOR_6_ADDR))
        return FLASH_SECTOR_5;
    if((addr >= FLASH_SECTOR_6_ADDR) && (addr < FLASH_SECTOR_7_ADDR))
        return FLASH_SECTOR_6;
    if((addr >= FLASH_SECTOR_7_ADDR) && (addr < FLASH_SECTOR_8_ADDR))
        return FLASH_SECTOR_7;
    if((addr >= FLASH_SECTOR_8_ADDR) && (addr < FLASH_SECTOR_9_ADDR))
        return FLASH_SECTOR_8;
    if((addr >= FLASH_SECTOR_9_ADDR) && (addr < FLASH_SECTOR_10_ADDR))
        return FLASH_SECTOR_9;
    if((addr >= FLASH_SECTOR_10_ADDR) && (addr < FLASH_SECTOR_11_ADDR))
        return FLASH_SECTOR_10;

    return FLASH_SECTOR_11;
}


static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;  
  }
  else if((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;  
  }
   // printf("Sector Address %d\n\r",sector);

  return sector;
}
