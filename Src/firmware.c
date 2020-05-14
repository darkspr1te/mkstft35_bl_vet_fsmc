#include <stddef.h>
#include <string.h>

#include "crc32.h"
#include "fatfs.h"
#include "flash_file.h"
#include "ff.h"
//#include "firmware_conf.h"
#include "firmware.h"
#define BOOT_ADDR FLASH_SECTOR_3_ADDR
#define APP_ADDR             BOOT_ADDR
//#define APP_ADDR             BOOT_ADDR
//#define HEADER_ADDR          (APP_ADDR - HEADER_SIZE)
#define HEADER_ADDR          APP_ADDR
extern SD_HandleTypeDef hsd;
extern SRAM_HandleTypeDef hsram1;
typedef void (*pFunction)(void);
static uint8_t upload_file(const char* name);
static uint8_t parse_header(uint8_t *data, FirmwareHeader_t* header);
static uint32_t GetSector(uint32_t Address);
//#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_2 
FATFS fs;

extern char SDPath[4];
extern FATFS SDFatFS;
extern FIL SDFile;

uint8_t firmware_init(void) {
    FRESULT res;
    printf("FLASH Enable memory\n\r");
    memset(&fs, 0, sizeof(fs));
    res = f_mount(DRIVE_NO, &fs,1);
    if (res) return 0;

    return 1;
}

uint8_t firmware_deinit(void) {
    FRESULT res;

    res = f_mount(DRIVE_NO, NULL,1);
    if (res) return 0;

    return 1;
}

uint16_t firmware_curr_version(void) {
    uint8_t res;
    FirmwareHeader_t header;

    res = parse_header((uint8_t *) HEADER_ADDR, &header);
    if (res != 1) return 0;

    return header.version;
}

uint16_t firmware_new_version(void) {
    FRESULT res;
    FIL file;
    UINT bytes_read;
    uint8_t buffer[HEADER_SIZE];
    FirmwareHeader_t header;

    memset(&file, 0, sizeof(file));
    res = f_open(&file, NEW_NAME, FA_OPEN_EXISTING | FA_READ);
    if (res) return 0;

    res = f_read(&file, (void*) buffer, HEADER_SIZE, &bytes_read);
    if (res || bytes_read < HEADER_SIZE) return 0;

    res = f_close(&file);
    if (res) return 0;

    res = parse_header(buffer, &header);
    if (res != 1) return 0;

    return header.version;
}

uint8_t firmware_dump(void) {
    FRESULT res;
    FIL file;
    UINT bytes_written;
    FirmwareHeader_t header;
    //skips ver check
    return 1;
    
    res = parse_header((uint8_t *) HEADER_ADDR, &header);
    if (res != 1) return 0;

    uint32_t file_size = HEADER_SIZE + header.size;

    memset(&file, 0, sizeof(file));
    res = f_open(&file, CURR_NAME, FA_OPEN_ALWAYS | FA_WRITE);
    if (res) return 0;

    res = f_write(&file, (uint8_t *) HEADER_ADDR, file_size, &bytes_written);
    if (res || bytes_written < file_size) return 0;

    res = f_close(&file);
    if (res) return 0;

    return 1;
}

uint8_t firmware_update(void) {
    return upload_file(NEW_NAME);
}

uint8_t firmware_restore(void) {
    return upload_file(CURR_NAME);
}

uint8_t firmware_basic(void) {
    return upload_file(BASIC_NAME);
}

uint8_t firmware_integrity(void) {
    uint32_t crc;
    uint8_t res;
    FirmwareHeader_t header;
    //skips crc checks
    return 1;

    res = parse_header((uint8_t *) HEADER_ADDR, &header);
    if (res != 1) return 0;

    uint8_t *addr = (uint8_t *) APP_ADDR;

    crc32_init(&crc);
    crc32_next(&crc, addr, header.size);
    crc32_res(&crc);

    return (crc == header.crc) ? 1 : 0;
}

void firmware_run(void) {
    
    printf("\n\rworking with un-encrypted mks firmware_run\n\r");
    uint32_t appStack = (uint32_t) *((uint32_t *) APP_ADDR);

    pFunction appEntry = (pFunction) *(uint32_t *) (APP_ADDR + 4);
    printf("Stack Address %x \n\r",APP_ADDR);
    printf("Jump Address %x\n\r",(APP_ADDR+4));
    printf("Stack Address Value %x \n\r",appStack);
    printf("Jump Address Value %x\n\r",appEntry);
    HAL_Delay(300);
  /* 
 HAL_Init();
  SystemClock_Config();
MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_FSMC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  int error_lcd = BSP_LCD_Init();
  BSP_LCD_Clear(LCD_COLOR_BLUE);
*/
 //try 1 
  // HAL_SRAM_MspDeInit(&hsram1);
   HAL_SD_MspDeInit(&hsd);
   HAL_RCC_DeInit();
    HAL_DeInit();
    //try 2 HAL_SD_DeInit(&hsd);
/*
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_SDIO_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
     __HAL_RCC_SYSCFG_CLK_DISABLE();
  __HAL_RCC_PWR_CLK_DISABLE();*/
    SCB->VTOR = APP_ADDR& 0x1FFFFF80;;
    SysTick->CTRL = 0;
    __set_MSP(appStack);
    //printf("Stack Address %x Value:%x\n\r",APP_ADDR,appStack);
   // printf("Stack Address %x Value %x\n\r",(APP_ADDR+4),appEntry);

    appEntry();
}

static uint8_t upload_file(const char* name) {
    FRESULT res;
    FIL file;
    UINT bytes_read;
    uint8_t flash_res;
    uint8_t buffer[1024];
    uint32_t addr = HEADER_ADDR;
    
    memset(&file, 0, sizeof(file));
    //res = f_mount(&file, name, 1);
    res = f_mount(&SDFatFS, SDPath, 1);
    printf("FLASH mount file, filename %s and error code %d\n\r",name,res);
    printf("about to loadfile\n\r");
    //res = f_open(&file, name, FA_OPEN_EXISTING | FA_READ);
   res = f_open(&SDFile, name, FA_OPEN_EXISTING | FA_READ);
    //printf("load file complete\n\r");
    if (res>=1)
    {
        printf("Flash opened code:%d\n\r",res);
    }
    else 
    {
      printf("load file complete\n\r");  
    }
    //if (res) return res;

   // flash_res = flash_clear(HEADER_ADDR, FLASH_LAST_ADDR);
  // printf("About to erase flash\n\r");
    //flash_res = flash_erase(FLASH_SECTOR_3_ADDR, FLASH_LAST_ADDR);
    HAL_FLASH_Unlock();
    flash_res = flash_erase(ADDR_FLASH_SECTOR_3,ADDR_FLASH_SECTOR_10);
    printf("erase flash complete code:%d\n\r",flash_res);
   // if (flash_res == 0) return 0;
   // HAL_FLASH_Unlock();
    addr = ADDR_FLASH_SECTOR_3;
  //  memset(&file, 0, sizeof(file));
   // res = f_mount(&file, name, 1);
  //  res = f_open(&file, name, FA_OPEN_EXISTING | FA_READ);



  

  while (addr < ADDR_FLASH_SECTOR_10)
  {
      res = f_read(&SDFile, buffer, sizeof(buffer), &bytes_read);
      if (res != FR_OK)
      {
          printf("error loading %d",res);
      }
    if (flash_program_by_word(addr, (uint32_t *) buffer, bytes_read / 4) == HAL_OK)
    {
      addr = addr + 4;
    }
    else
    { 
      
       return 99;
      Error_Handler();
    }

  } 

/*
    while (1) {
        printf("firmware.c -read bin file %d\n\r",res);
        res = f_read(&SDFile, buffer, sizeof(buffer), &bytes_read);
        printf("firmware.c -bin file read buffer %d bytes read %d\n\r",res,bytes_read);
        if (res != 0)
        {
            printf("firmware.c -read bin file error %d\n\r",res);
        }
        
       // if (res) return 0;
        if (bytes_read < sizeof(buffer)) 
        {
            printf("buffer error\n\r");
            break;
        }
        //printf("firmware.c -write bin file\n\r");
        addr = flash_program_by_word(addr, (uint32_t *) buffer, bytes_read / 4);
        printf("firmware.c - flashing address word %x\n\r",addr);
        if (addr == 0) return 0;
    }

    

    if (bytes_read > 0) {
        addr = flash_program_by_byte(addr, buffer, bytes_read);
        printf("firmware.c flashing address byte %x\n\r",addr);
        if (addr == 0) return 0;
    }
*/
    res = f_close(&file);
    printf("Flashing upload finished %d\n\r",res);
    if (res) return 0;
    HAL_FLASH_Lock();
    return 1;
}

static uint8_t parse_header(uint8_t *data, FirmwareHeader_t* header) {
    header->entry_sign = *(data + ENTRY_OFFSET);
    header->crc = *(uint32_t *) (data + CRC_OFFSET);
    header->crc_shadow = *(uint32_t *) (data + CRC_SHADOW_OFFSET);
    header->size = *(uint32_t *) (data + SIZE_OFFSET);
    header->uuid = *(uint32_t *) (data + UUID_OFFSET);
    header->version = *(uint16_t *) (data + VERSION_OFFSET);
    header->closing_sign = *(data + CLOSE_OFFSET);

    if (header->entry_sign != ENTRY_VALUE) return 0;
    if (header->closing_sign != CLOSE_VALUE) return 0;

    return 1;
}
