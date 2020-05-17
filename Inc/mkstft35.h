#ifndef __MKSTFT_H__
#define __MKSTFT_H__
#ifdef __cplusplus
 extern "C" {
#endif

#define SOFTWARE_VERSION "0.1"
#define LOADER_VARIANT "iz3man loader"
#define HARDWARE "MKSTFT3.5-V1"
#define FIRMWARE_FILENAME_NOCRYPT "mkstf35-u.bin\0"
#define FIRMWARE_FILENAME_CRYPT  "mkstf35.bin\0"
#define LOCATION                "0:/"
#define OLDFIRMWARE      LOCATION FIRMWARE_FILENAME
#define OLD                     "old"
#define RENAME_FILE        LOCATION OLD FIRMWARE_FILENAME

#if !defined(MAIN_PR_OFFSET)
#define MAIN_PR_OFFSET 0xC000
#endif
#if !defined(FIRMWARE)
#define FIRMWARE                "0:/mkstft35.bin"
#endif 

//typedef void (*Callable)();

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define _SECTOR_SIZE 0x3fff
#define FLASH_PAGE_SIZE 0x3fff
#define BOOT_SECT 0xC000
#define FLASH_SIZE (_SECTOR_SIZE*FLASH_SECTOR_TOTAL)


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
