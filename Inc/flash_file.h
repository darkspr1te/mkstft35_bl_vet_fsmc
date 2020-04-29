#ifndef __FLASH_H
#define __FLASH_H
#include "stm32f4xx.h"
#include "stdint.h"
typedef enum
{
	FLASH_RESULT_OK = 0,
	FLASH_RESULT_FILE_ERROR,
	FLASH_RESULT_FLASH_ERROR,
	FLASH_FILE_NOT_EXISTS,
	FLASH_FILE_CANNOT_OPEN,
	FLASH_FILE_INVALID_HASH,
	FLASH_FILE_TOO_BIG
} FlashResult;

FlashResult flash(const char *fname);
//extern const uint32_t *mcuFirstPageAddr;

typedef void (*Callable)();
#endif /* __BOOT_CONF_H */