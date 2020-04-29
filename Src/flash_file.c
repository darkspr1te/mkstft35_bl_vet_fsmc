#include "mkstft35.h"
#include "ff.h"
#include "stm32f4xx_hal.h"
#include "flash_file.h"
#define FLASH_PAGE_SIZE FLASH_SECTOR_TOTAL

extern unsigned int _isr_real;
extern const uint32_t *mcuFirstPageAddr;

//uint32_t mcuLastPageAddr = ((uint32_t) &_isr_real) - FLASH_PAGE_SIZE;

uint8_t buffer[FLASH_PAGE_SIZE];
uint32_t bufferLen = 0;

static FIL fil;
static FILINFO info;
FLASH_EraseInitTypeDef erase_flash;
//HAL_StatusTypeDef erase(uint32_t from, uint32_t to)
HAL_StatusTypeDef erase(void)
{
	HAL_StatusTypeDef res = HAL_OK;


/*
	for (uint32_t i = from; i <= to; i += FLASH_PAGE_SIZE)
	{
		FLASH_EraseInitTypeDef erase;

		erase.TypeErase = FLASH_TYPEERASE_PAGES;
		erase.Banks = FLASH_BANK_1;
		erase.PageAddress = i;
		erase.NbPages = 1;

		uint32_t error = 0;
		res = HAL_FLASHEx_Erase(&erase, &error);
#ifdef DEBUG		
		printf("erase page %#010x\n\r",i);
#endif
		if (res != HAL_OK) {
			return res;
		}
	}
*/
//unit32_t address = 0x0800C000;
HAL_FLASH_Unlock();
__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
for (int x=FLASH_SECTOR_3;x<FLASH_SECTOR_TOTAL;x++)
{
    FLASH_Erase_Sector(x, VOLTAGE_RANGE_3);
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
		res = f_read(&fil, target, fileBlock, &readNow);

		target += readNow;
		*read += readNow;

		if (res != FR_OK || readNow != fileBlock)
			break;
	}
	return res;
}

HAL_StatusTypeDef flashWrite(uint32_t position, uint8_t *data, uint32_t size)
{
	#ifdef DEBUG	
		printf("write page %#010x\n\r",position);
		#endif
	HAL_StatusTypeDef res = HAL_OK;
	for (uint32_t i=0; i<size; i+=2)
	{
		
		res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, position + i, *(uint16_t*)&data[i]);
		if (res != HAL_OK)
			break;
	}
	return res;
}

FlashResult flash(const char *fname)
{
	FRESULT res = f_stat(fname, &info);
	if (res != FR_OK)
		return FLASH_FILE_NOT_EXISTS;

	// Checking file size
//	if (info.fsize > getMaxFlashImageSize())
//		return FLASH_FILE_TOO_BIG;

	res = f_open(&fil, fname, FA_OPEN_EXISTING | FA_READ);
if (res == FR_OK)
  {
#ifdef DEBUG		  
	  printf("flash open ok\n\r");
#endif
  }
	if (res != FR_OK)
	{
#ifdef DEBUG	
	  printf("flash open file failed\n\r");
#endif
		return FLASH_RESULT_FILE_ERROR;
	}
	uint32_t position = (uint32_t) mcuFirstPageAddr;

	if (HAL_OK != HAL_FLASH_Unlock())
		return FLASH_RESULT_FLASH_ERROR;
#ifdef DEBUG			
	printf("flash unlock\n\r");
#endif
	if (HAL_OK != erase())
    //erase((uint32_t) mcuFirstPageAddr, info.fsize + (uint32_t)mcuFirstPageAddr))
	{
#ifdef DEBUG	
	printf("erase error\n\r");
#endif
		return FLASH_RESULT_FLASH_ERROR;
	}

	do {
		readNextPage(buffer, &bufferLen);
		
		if (HAL_OK != flashWrite(position, buffer, bufferLen))
			return FLASH_RESULT_FLASH_ERROR;

		position += bufferLen;
	} while (bufferLen != 0);

	f_close(&fil);
	HAL_FLASH_Lock();
#ifdef DEBUG
	  printf("flash write complete \r\n");
#endif
	return FR_OK;
}

