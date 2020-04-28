/*
 * SPI LCD driver STM32F1
 * author: Roberto Benjami
 * version:  2020.01.21
 *
 * - hardware, software SPI
 * - 3 modes (only TX, half duplex, full duplex)
*/

//-----------------------------------------------------------------------------
#include "main.h"
#include "lcd.h"
#include "lcd_io_spi.h"

#define DMA_MAXSIZE           0xFFFE

//-----------------------------------------------------------------------------
/* Link function for LCD peripheral */
void  LCD_Delay (uint32_t delay);
void  LCD_IO_Init(void);
void  LCD_IO_Bl_OnOff(uint8_t Bl);

void  LCD_IO_WriteCmd8(uint8_t Cmd);
void  LCD_IO_WriteCmd16(uint16_t Cmd);
void  LCD_IO_WriteData8(uint8_t Data);
void  LCD_IO_WriteData16(uint16_t Data);

void  LCD_IO_WriteCmd8DataFill16(uint8_t Cmd, uint16_t Data, uint32_t Size);
void  LCD_IO_WriteCmd8MultipleData8(uint8_t Cmd, uint8_t *pData, uint32_t Size);
void  LCD_IO_WriteCmd8MultipleData16(uint8_t Cmd, uint16_t *pData, uint32_t Size);
void  LCD_IO_WriteCmd16DataFill16(uint16_t Cmd, uint16_t Data, uint32_t Size);
void  LCD_IO_WriteCmd16MultipleData8(uint16_t Cmd, uint8_t *pData, uint32_t Size);
void  LCD_IO_WriteCmd16MultipleData16(uint16_t Cmd, uint16_t *pData, uint32_t Size);

void  LCD_IO_ReadCmd8MultipleData8(uint8_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize);
void  LCD_IO_ReadCmd8MultipleData16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);
void  LCD_IO_ReadCmd8MultipleData24to16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);
void  LCD_IO_ReadCmd16MultipleData8(uint16_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize);
void  LCD_IO_ReadCmd16MultipleData16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);
void  LCD_IO_ReadCmd16MultipleData24to16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize);

void  LCD_IO_Delay(uint32_t c);

//=============================================================================

#define BITBAND_ACCESS(a, b)  *(volatile uint32_t*)(((uint32_t)&a & 0xF0000000) + 0x2000000 + (((uint32_t)&a & 0x000FFFFF) << 5) + (b << 2))

//-----------------------------------------------------------------------------
/* GPIO mode */
#define MODE_ANALOG_INPUT     0x0
#define MODE_PP_OUT_10MHZ     0x1
#define MODE_PP_OUT_2MHZ      0x2
#define MODE_PP_OUT_50MHZ     0x3
#define MODE_FF_DIGITAL_INPUT 0x4
#define MODE_OD_OUT_10MHZ     0x5
#define MODE_OD_OUT_2MHZ      0x6
#define MODE_OD_OUT_50MHZ     0x7
#define MODE_PU_DIGITAL_INPUT 0x8
#define MODE_PP_ALTER_10MHZ   0x9
#define MODE_PP_ALTER_2MHZ    0xA
#define MODE_PP_ALTER_50MHZ   0xB
#define MODE_RESERVED         0xC
#define MODE_OD_ALTER_10MHZ   0xD
#define MODE_OD_ALTER_2MHZ    0xE
#define MODE_OD_ALTER_50MHZ   0xF

#define GPIOX_PORT_(a, b)     GPIO ## a
#define GPIOX_PORT(a)         GPIOX_PORT_(a)

#define GPIOX_PIN_(a, b)      b
#define GPIOX_PIN(a)          GPIOX_PIN_(a)

#define GPIOX_MODE_(a,b,c)    ((GPIO_TypeDef*)(((c & 8) >> 1) + GPIO ## b ## _BASE))->CRL = (((GPIO_TypeDef*)(((c & 8) >> 1) + GPIO ## b ## _BASE))->CRL & ~(0xF << ((c & 7) << 2))) | (a << ((c & 7) << 2))
#define GPIOX_MODE(a, b)      GPIOX_MODE_(a, b)

#define GPIOX_ODR_(a, b)      BITBAND_ACCESS(GPIO ## a ->ODR, b)
#define GPIOX_ODR(a)          GPIOX_ODR_(a)

#define GPIOX_IDR_(a, b)      BITBAND_ACCESS(GPIO ## a ->IDR, b)
#define GPIOX_IDR(a)          GPIOX_IDR_(a)

#define GPIOX_LINE_(a, b)     EXTI_Line ## b
#define GPIOX_LINE(a)         GPIOX_LINE_(a)

#define GPIOX_PORTSRC_(a, b)  GPIO_PortSourceGPIO ## a
#define GPIOX_PORTSRC(a)      GPIOX_PORTSRC_(a)

#define GPIOX_PINSRC_(a, b)   GPIO_PinSource ## b
#define GPIOX_PINSRC(a)       GPIOX_PINSRC_(a)

#define GPIOX_CLOCK_(a, b)    RCC_APB2ENR_IOP ## a ## EN
#define GPIOX_CLOCK(a)        GPIOX_CLOCK_(a)

#define GPIOX_PORTNUM_A       1
#define GPIOX_PORTNUM_B       2
#define GPIOX_PORTNUM_C       3
#define GPIOX_PORTNUM_D       4
#define GPIOX_PORTNUM_E       5
#define GPIOX_PORTNUM_F       6
#define GPIOX_PORTNUM_G       7
#define GPIOX_PORTNUM_H       8
#define GPIOX_PORTNUM_I       9
#define GPIOX_PORTNUM_J       10
#define GPIOX_PORTNUM_K       11
#define GPIOX_PORTNUM_(a, b)  GPIOX_PORTNUM_ ## a
#define GPIOX_PORTNUM(a)      GPIOX_PORTNUM_(a)

#define GPIOX_PORTNAME_(a, b) a
#define GPIOX_PORTNAME(a)     GPIOX_PORTNAME_(a)

//-----------------------------------------------------------------------------
/* DMA */
#define DMANUM_(a, b, c)                a
#define DMANUM(a)                       DMANUM_(a)

#define DMACHN_(a, b, c)                b
#define DMACHN(a)                       DMACHN_(a)

#define DMAPRIORITY_(a, b, c)           c
#define DMAPRIORITY(a)                  DMAPRIORITY_(a)

#define DMAX_(a, b, c)                  DMA ## a
#define DMAX(a)                         DMAX_(a)

#define DMAX_CHANNEL_(a, b, c)          DMA ## a ## _Channel ## b
#define DMAX_CHANNEL(a)                 DMAX_CHANNEL_(a)

#define DMAX_CHANNEL_IRQ_(a, b, c)      DMA ## a ## _Channel ## b ## _IRQn
#define DMAX_CHANNEL_IRQ(a)             DMAX_CHANNEL_IRQ_(a)

#define DMAX_CHANNEL_IRQHANDLER_(a, b, c) DMA ## a ## _Channel ## b ## _IRQHandler
#define DMAX_CHANNEL_IRQHANDLER(a)      DMAX_CHANNEL_IRQHANDLER_(a)

// Interrupt event pl: if(DMAX_ISR(LCD_DMA_TX) & DMAX_ISR_TCIF(LCD_DMA_TX))...
#define DMAX_ISR_TCIF_(a, b, c)         DMA_ISR_TCIF ## b
#define DMAX_ISR_TCIF(a)                DMAX_ISR_TCIF_(a)

#define DMAX_ISR_HTIF_(a, b, c)         DMA_ISR_HTIF ## b
#define DMAX_ISR_HTIF(a)                DMAX_ISR_HTIF_(a)

#define DMAX_ISR_TEIF_(a, b, c)         DMA_ISR_TEIF ## b
#define DMAX_ISR_TEIF(a)                DMAX_ISR_TEIF_(a)

#define DMAX_ISR_GIF_(a, b, c)          DMA_ISR_GIF ## b
#define DMAX_ISR_GIF(a)                 DMAX_ISR_GIF_(a)

// Interrupt clear pl: DMAX_IFCR(LCD_DMA_TX) = DMAX_IFCR_CTCIF(LCD_DMA_TX) | DMAX_IFCR_CFEIF(LCD_DMA_TX);
#define DMAX_IFCR_CTCIF_(a, b, c)       DMA_IFCR_CTCIF ## b
#define DMAX_IFCR_CTCIF(a)              DMAX_IFCR_CTCIF_(a)

#define DMAX_IFCR_CHTIF_(a, b, c)       DMA_IFCR_CHTIF ## b
#define DMAX_IFCR_CHTIF(a)              DMAX_IFCR_CHTIF_(a)

#define DMAX_IFCR_CTEIF_(a, b, c)       DMA_IFCR_CTEIF ## b
#define DMAX_IFCR_CTEIF(a)              DMAX_IFCR_CTEIF_(a)

#define DMAX_IFCR_CGIF_(a, b, c)        DMA_IFCR_CGIF ## b
#define DMAX_IFCR_CGIF(a)               DMAX_IFCR_CGIF_(a)

//=============================================================================
/* Command/data pin set */
#define LCD_RS_CMD            GPIOX_ODR(LCD_RS) = 0
#define LCD_RS_DATA           GPIOX_ODR(LCD_RS) = 1

/* Reset pin set */
#define LCD_RST_ON            GPIOX_ODR(LCD_RST) = 0
#define LCD_RST_OFF           GPIOX_ODR(LCD_RST) = 1

/* Chip select pin set */
#define LCD_CS_ON             GPIOX_ODR(LCD_CS) = 0
#define LCD_CS_OFF            GPIOX_ODR(LCD_CS) = 1

/* If the read speed is undefinied -> is the same as writing speed */
#ifndef LCD_SPI_SPD_READ
#define LCD_SPI_SPD_READ      LCD_SPI_SPD_WRITE
#endif

/* Write SPI delay (0: none, 1: NOP, 2: clock pin keeping, 3.. LCD_IO_Delay) */
#if     LCD_SPI_SPD_WRITE == 0
#define LCD_WRITE_DELAY
#elif   LCD_SPI_SPD_WRITE == 1
#define LCD_WRITE_DELAY       __NOP()
#elif   LCD_SPI_SPD_WRITE == 2
#define LCD_WRITE_DELAY       GPIOX_ODR(LCD_SCK) = 0
#else
#define LCD_WRITE_DELAY       LCD_IO_Delay(LCD_SPI_SPD_WRITE - 3)
#endif

/* Read SPI delay (0: none, 1: NOP, 2: clock pin keeping, 3.. LCD_IO_Delay) */
#if     LCD_SPI_SPD_READ == 0
#define LCD_READ_DELAY
#elif   LCD_SPI_SPD_READ == 1
#define LCD_READ_DELAY        __NOP()
#elif   LCD_SPI_SPD_READ == 2
#define LCD_READ_DELAY        GPIOX_ODR(LCD_SCK) = 0
#else
#define LCD_READ_DELAY        LCD_IO_Delay(LCD_SPI_SPD_READ - 3)
#endif

#if GPIOX_PORTNUM(LCD_RS) < GPIOX_PORTNUM_A
#error  not definied the LCD RS pin
#endif

#if GPIOX_PORTNUM(LCD_CS) < GPIOX_PORTNUM_A
#error  not definied the LCD CS pin
#endif

#if GPIOX_PORTNUM(LCD_SCK) < GPIOX_PORTNUM_A
#error  not definied the LCD SCK pin
#endif

#if GPIOX_PORTNUM(LCD_MOSI) < GPIOX_PORTNUM_A
#error  not definied the LCD MOSI pin
#endif

#if GPIOX_PORTNUM(LCD_MISO) < GPIOX_PORTNUM_A && LCD_SPI_MODE == 2
#error  not definied the LCD MISO pin
#endif

//-----------------------------------------------------------------------------
#if LCD_SPI == 0
/* Software SPI */

#define LcdSpiMode8()
#define LcdSpiMode16()

#define LCD_WRITE_CLK         GPIOX_ODR(LCD_SCK) = 0; LCD_WRITE_DELAY; GPIOX_ODR(LCD_SCK) = 1;
#define LCD_READ_CLK          GPIOX_ODR(LCD_SCK) = 1; GPIOX_ODR(LCD_SCK) = 0; LCD_READ_DELAY;

void LcdWrite8(uint8_t d8)
{
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 7);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 6);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 5);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 4);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 3);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 2);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 1);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d8, 0);
  LCD_WRITE_CLK;
}

void LcdWrite16(uint16_t d16)
{
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 15);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 14);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 13);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 12);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 11);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 10);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 9);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 8);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 7);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 6);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 5);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 4);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 3);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 2);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 1);
  LCD_WRITE_CLK;
  GPIOX_ODR(LCD_MOSI) = BITBAND_ACCESS(d16, 0);
  LCD_WRITE_CLK;
}

#define LcdCmdWrite8(cmd8)      {LCD_RS_CMD; LcdWrite8(cmd8); LCD_RS_DATA;}
#define LcdCmdWrite16(cmd16)    {LCD_RS_CMD; LcdWrite16(cmd16); LCD_RS_DATA;}

#if LCD_SPI_MODE != 0
/* half duplex, full duplex */

#if     LCD_SPI_MODE == 1
/* Halfduplex mode : data direction change and MISO pin = MOSI pin */
#undef  LCD_MISO
#define LCD_MISO              LCD_MOSI
#define LcdDirWrite()         GPIOX_MODE(MODE_PP_OUT_50MHZ, LCD_MOSI)

#elif   LCD_SPI_MODE == 2
/* Fullduplex SPI : data direction is fix, dummy read */
#define LcdDirWrite()
#endif  // #elif   LCD_SPI_MODE == 2

//-----------------------------------------------------------------------------
void LcdDirRead(uint32_t d)
{
  #if LCD_SPI_MODE == 1
  GPIOX_MODE(MODE_FF_DIGITAL_INPUT, LCD_MOSI);
  #endif
  LCD_READ_DELAY;
  while(d--)
  {
    GPIOX_ODR(LCD_SCK) = 0;
    LCD_READ_DELAY;
    GPIOX_ODR(LCD_SCK) = 1;
  }
}

//-----------------------------------------------------------------------------
uint8_t LcdRead8(void)
{
  uint8_t d8;
  GPIOX_ODR(LCD_SCK) = 0;
  LCD_READ_DELAY;
  BITBAND_ACCESS(d8, 7) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d8, 6) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d8, 5) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d8, 4) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d8, 3) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d8, 2) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d8, 1) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d8, 0) = GPIOX_IDR(LCD_MISO);
  GPIOX_ODR(LCD_SCK) = 1;
  return d8;
}

//-----------------------------------------------------------------------------
uint16_t LcdRead16(void)
{
  uint16_t d16;
  GPIOX_ODR(LCD_SCK) = 0;
  LCD_READ_DELAY;
  BITBAND_ACCESS(d16, 15) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 14) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 13) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 12) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 11) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 10) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 9) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 8) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 7) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 6) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 5) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 4) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 3) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 2) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 1) = GPIOX_IDR(LCD_MISO);
  LCD_READ_CLK;
  BITBAND_ACCESS(d16, 0) = GPIOX_IDR(LCD_MISO);
  GPIOX_ODR(LCD_SCK) = 1;
  return d16;
}
#endif  // #if LCD_SPI_MODE != 0

/* not using the DMA -> no need to wait for the end of the previous DMA operation */

//=============================================================================
#else    // #if LCD_SPI == 0
/* Hardware SPI */

#if LCD_SPI == 1
#define SPIX                  SPI1
#define LCD_SPI_RCC_EN        BITBAND_ACCESS(RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Pos) = 1
#elif LCD_SPI == 2
#define SPIX                  SPI2
#define LCD_SPI_RCC_EN        BITBAND_ACCESS(RCC->APB1ENR, RCC_APB1ENR_SPI2EN_Pos) = 1
#elif LCD_SPI == 3
#define SPIX                  SPI3
#define LCD_SPI_RCC_EN        BITBAND_ACCESS(RCC->APB1ENR, RCC_APB1ENR_SPI3EN_Pos) = 1
#endif

#define LcdSpiMode8()         BITBAND_ACCESS(SPIX->CR1, SPI_CR1_DFF_Pos) = 0
#define LcdSpiMode16()        BITBAND_ACCESS(SPIX->CR1, SPI_CR1_DFF_Pos) = 1

//-----------------------------------------------------------------------------
#if     LCD_SPI_MODE == 1
/* Halfduplex SPI : the direction of the data must be changed */

/* If -O0 optimize: in compiler -> inline functions are error. This is the solution */
extern inline void LcdDirRead(uint32_t d);

/* Data direction from OUT to IN. The parameter: dummy clock number */
inline void LcdDirRead(uint32_t d)
{
  GPIOX_MODE(MODE_PP_OUT_50MHZ, LCD_SCK);
  while(d--)
  {
    GPIOX_ODR(LCD_SCK) = 0;
    LCD_READ_DELAY;
    GPIOX_ODR(LCD_SCK) = 1;
  }
  GPIOX_MODE(MODE_PP_ALTER_50MHZ, LCD_SCK);
  SPIX->CR1 = (SPIX->CR1 & ~(SPI_CR1_BR | SPI_CR1_BIDIOE)) | (LCD_SPI_SPD_READ << SPI_CR1_BR_Pos);
}

/* Data direction from IN to OUT */
extern inline void LcdDirWrite(void);
inline void LcdDirWrite(void)
{
  volatile uint8_t d8 __attribute__((unused));
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos))
    d8 = SPIX->DR;
  SPIX->CR1 &= ~SPI_CR1_SPE;
  SPIX->CR1 = (SPIX->CR1 & ~SPI_CR1_BR) | ((LCD_SPI_SPD_WRITE << SPI_CR1_BR_Pos) | SPI_CR1_BIDIOE);
  LCD_IO_Delay(2 ^ LCD_SPI_SPD_READ);
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos))
    d8 = SPIX->DR;
  SPIX->CR1 |= SPI_CR1_SPE;
}

#elif   LCD_SPI_MODE == 2
/* Fullduplex SPI : the direction is fix */
extern inline void LcdDirRead(uint32_t d);
inline void LcdDirRead(uint32_t d)
{
  GPIOX_MODE(MODE_PP_OUT_50MHZ, LCD_SCK);
  while(d--)
  {
    GPIOX_ODR(LCD_SCK) = 0;
    LCD_READ_DELAY;
    GPIOX_ODR(LCD_SCK) = 1;
  }
  GPIOX_MODE(MODE_OD_ALTER_50MHZ, LCD_SCK);
  SPIX->CR1 = (SPIX->CR1 & ~SPI_CR1_BR) | (LCD_SPI_SPD_READ << SPI_CR1_BR_Pos);
}

extern inline void LcdDirWrite(void);
inline void LcdDirWrite(void)
{
  SPIX->CR1 = (SPIX->CR1 & ~SPI_CR1_BR) | (LCD_SPI_SPD_WRITE << SPI_CR1_BR_Pos);
}

#endif

//-----------------------------------------------------------------------------
extern inline void LcdWrite8(uint8_t d8);
inline void LcdWrite8(uint8_t d8)
{
  SPIX->DR = d8;
  LCD_IO_Delay(2);
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_BSY_Pos));
}

//-----------------------------------------------------------------------------
extern inline uint8_t LcdRead8(void);
inline uint8_t LcdRead8(void)
{
  uint8_t d8;
  while(!BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos));
  d8 = (uint8_t)SPIX->DR;
  return d8;
}

//-----------------------------------------------------------------------------
extern inline void LcdCmdWrite8(uint8_t cmd8);
inline void LcdCmdWrite8(uint8_t cmd8)
{
  LCD_RS_CMD;
  SPIX->DR = cmd8;
  LCD_IO_Delay(2);
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_BSY_Pos));
  LCD_RS_DATA;
}

//-----------------------------------------------------------------------------
extern inline void LcdWrite16(uint16_t d16);
inline void LcdWrite16(uint16_t d16)
{
  SPIX->DR = d16;
  LCD_IO_Delay(1);
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_BSY_Pos));
}

//-----------------------------------------------------------------------------
extern inline uint16_t LcdRead16(void);
inline uint16_t LcdRead16(void)
{
  uint16_t d16;
  while(!BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos));
  d16 = SPIX->DR;
  return d16;
}

//-----------------------------------------------------------------------------
extern inline void LcdCmdWrite16(uint16_t cmd16);
inline void LcdCmdWrite16(uint16_t cmd16)
{
  LCD_RS_CMD;
  SPIX->DR = cmd16;
  LCD_IO_Delay(2);
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_BSY_Pos));
  LCD_RS_DATA;
}

#endif   // #else LCD_SPI == 0

//-----------------------------------------------------------------------------
#if (DMANUM(LCD_DMA_TX) > 0 || DMANUM(LCD_DMA_RX) > 0) && LCD_SPI > 0
/* DMA transfer end check */

/* DMA status
   - 0: all DMA transfers are completed
   - 1: last DMA transfer in progress
   - 2: DMA transfer in progress */
volatile uint32_t LCD_IO_DmaTransferStatus = 0;

//-----------------------------------------------------------------------------
/* Waiting for all DMA processes to complete */
#ifndef osFeature_Semaphore
/* no FreeRtos */

extern inline void WaitForDmaEnd(void);
inline void WaitForDmaEnd(void)
{
  while(LCD_IO_DmaTransferStatus);
}

//-----------------------------------------------------------------------------
#else   // osFeature_Semaphore
/* FreeRtos */

osSemaphoreId spiDmaBinSemHandle;

extern inline void WaitForDmaEnd(void);
inline void WaitForDmaEnd(void)
{
  if(LCD_IO_DmaTransferStatus)
  {
    osSemaphoreWait(spiDmaBinSemHandle, 500);
    if(LCD_IO_DmaTransferStatus == 1)
      LCD_IO_DmaTransferStatus = 0;
  }
}

#endif  // #else osFeature_Semaphore

#else   // #if (DMANUM(LCD_DMA_TX) > 0 || DMANUM(LCD_DMA_RX) > 0) && LCD_SPI > 0

#define  WaitForDmaEnd()                /* if DMA is not used -> no need to wait */

#endif  // #else (DMANUM(LCD_DMA_TX) > 0 || DMANUM(LCD_DMA_RX) > 0) && LCD_SPI > 0

//-----------------------------------------------------------------------------
#if DMANUM(LCD_DMA_TX) == 0 || LCD_SPI == 0

/* SPI TX no DMA */

void LCD_IO_WriteMultiData8(uint8_t * pData, uint32_t Size, uint32_t dinc)
{
  while(Size--)
  {
    LcdWrite8(*pData);
    if(dinc)
      pData++;
  }
  LCD_CS_OFF;
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteMultiData16(uint16_t * pData, uint32_t Size, uint32_t dinc)
{
  while(Size--)
  {
    LcdWrite16(*pData);
    if(dinc)
      pData++;
  }
  LCD_CS_OFF;
}

#else // #if DMANUM(LCD_DMA_TX) == 0 || LCD_SPI == 0

//-----------------------------------------------------------------------------
/* SPI TX on DMA */

//-----------------------------------------------------------------------------
void DMAX_CHANNEL_IRQHANDLER(LCD_DMA_TX)(void)
{
  if(DMAX(LCD_DMA_TX)->ISR & DMAX_ISR_TCIF(LCD_DMA_TX))
  {
    DMAX(LCD_DMA_TX)->IFCR = DMAX_IFCR_CTCIF(LCD_DMA_TX);
    DMAX_CHANNEL(LCD_DMA_TX)->CCR = 0;
    while(DMAX_CHANNEL(LCD_DMA_TX)->CCR & DMA_CCR_EN);
    BITBAND_ACCESS(SPIX->CR2, SPI_CR2_TXDMAEN_Pos) = 0;
    while(BITBAND_ACCESS(SPIX->SR, SPI_SR_BSY_Pos));
    LCD_IO_Delay(2 ^ LCD_SPI_SPD_WRITE);

    if(LCD_IO_DmaTransferStatus == 1) /* last transfer end ? */
      LCD_CS_OFF;

    #ifndef osFeature_Semaphore
    /* no FreeRtos */
    LCD_IO_DmaTransferStatus = 0;
    #else
    /* FreeRtos */
    osSemaphoreRelease(spiDmaBinSemHandle);
    #endif // #else osFeature_Semaphore
  }
  else
    DMAX(LCD_DMA_TX)->IFCR = DMAX_IFCR_CGIF(LCD_DMA_TX);
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteMultiData(void * pData, uint32_t Size, uint32_t dmacr)
{
  DMAX(LCD_DMA_TX)->IFCR = DMAX_IFCR_CGIF(LCD_DMA_TX);
  SPIX->CR1 &= ~SPI_CR1_SPE;           /* SPI stop */
  DMAX_CHANNEL(LCD_DMA_TX)->CCR = 0;   /* DMA stop */
  while(DMAX_CHANNEL(LCD_DMA_TX)->CCR & DMA_CCR_EN);
  DMAX_CHANNEL(LCD_DMA_TX)->CMAR = (uint32_t)pData;
  DMAX_CHANNEL(LCD_DMA_TX)->CPAR = (uint32_t)&SPIX->DR;
  DMAX_CHANNEL(LCD_DMA_TX)->CNDTR = Size;
  DMAX_CHANNEL(LCD_DMA_TX)->CCR = dmacr;
  DMAX_CHANNEL(LCD_DMA_TX)->CCR |= DMA_CCR_EN;
  BITBAND_ACCESS(SPIX->CR2, SPI_CR2_TXDMAEN_Pos) = 1;
  SPIX->CR1 |= SPI_CR1_SPE;
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteMultiData8(uint8_t * pData, uint32_t Size, uint32_t dinc)
{
  uint32_t dmacr;
  static uint8_t d8s;
  if(!dinc)
  {
    d8s = *pData;
    pData = &d8s;
    dmacr = DMA_CCR_TCIE | (0 << DMA_CCR_MSIZE_Pos) |
            (0 << DMA_CCR_PSIZE_Pos) | DMA_CCR_DIR | (0 << DMA_CCR_MINC_Pos) |
            (DMAPRIORITY(LCD_DMA_TX) << DMA_CCR_PL_Pos);
  }
  else
    dmacr = DMA_CCR_TCIE | (0 << DMA_CCR_MSIZE_Pos) |
            (0 << DMA_CCR_PSIZE_Pos) | DMA_CCR_DIR | (1 << DMA_CCR_MINC_Pos) |
            (DMAPRIORITY(LCD_DMA_TX) << DMA_CCR_PL_Pos);

  while(Size)
  {
    if(Size <= DMA_MAXSIZE)
    {
      LCD_IO_DmaTransferStatus = 1;     /* last transfer */
      LCD_IO_WriteMultiData((void *)pData, Size, dmacr);
      Size = 0;
      #if LCD_DMA_TXWAIT == 1
      if(dinc)
        WaitForDmaEnd();
      #endif
    }
    else
    {
      LCD_IO_DmaTransferStatus = 2;     /* no last transfer */
      LCD_IO_WriteMultiData((void *)pData, DMA_MAXSIZE, dmacr);
      if(dinc)
        pData+= DMA_MAXSIZE;
      Size-= DMA_MAXSIZE;
      WaitForDmaEnd();
    }
  }
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteMultiData16(uint16_t * pData, uint32_t Size, uint32_t dinc)
{
  uint32_t dmacr;
  static uint16_t d16s;
  if(!dinc)
  {
    d16s = *pData;
    pData = &d16s;
    dmacr = DMA_CCR_TCIE | (1 << DMA_CCR_MSIZE_Pos) |
            (1 << DMA_CCR_PSIZE_Pos) | DMA_CCR_DIR | (0 << DMA_CCR_MINC_Pos) |
            (DMAPRIORITY(LCD_DMA_TX) << DMA_CCR_PL_Pos);
  }
  else
    dmacr = DMA_CCR_TCIE | (1 << DMA_CCR_MSIZE_Pos) |
            (1 << DMA_CCR_PSIZE_Pos) | DMA_CCR_DIR | (1 << DMA_CCR_MINC_Pos) |
            (DMAPRIORITY(LCD_DMA_TX) << DMA_CCR_PL_Pos);

  while(Size)
  {
    if(Size <= DMA_MAXSIZE)
    {
      LCD_IO_DmaTransferStatus = 1;     /* last transfer */
      LCD_IO_WriteMultiData((void *)pData, Size, dmacr);
      Size = 0;
      #if LCD_DMA_TXWAIT == 1
      if(dinc)
        WaitForDmaEnd();
      #endif
    }
    else if(Size < 2 * DMA_MAXSIZE)
    {
      LCD_IO_DmaTransferStatus = 2;     /* no last transfer */
      LCD_IO_WriteMultiData((void *)pData, Size - DMA_MAXSIZE, dmacr);
      if(dinc)
        pData+= Size - DMA_MAXSIZE;
      Size = DMA_MAXSIZE;
      WaitForDmaEnd();
    }
    else
    {
      LCD_IO_DmaTransferStatus = 2;     /* no last transfer */
      LCD_IO_WriteMultiData((void *)pData, DMA_MAXSIZE, dmacr);
      if(dinc)
        pData+= DMA_MAXSIZE;
      Size-= DMA_MAXSIZE;
      WaitForDmaEnd();
    }
  }
}

#endif // #else DMANUM(LCD_DMA_TX) == 0 || LCD_SPI == 0

//-----------------------------------------------------------------------------
#if LCD_SPI_MODE != 0
#if DMANUM(LCD_DMA_RX) == 0 || LCD_SPI == 0

void LCD_IO_ReadMultiData8(uint8_t * pData, uint32_t Size)
{
  uint8_t d8;
  while(Size--)
  {
    d8 = LcdRead8();
    *pData = d8;
    pData++;
  }
  LCD_CS_OFF;
  LcdDirWrite();
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadMultiData16(uint16_t * pData, uint32_t Size)
{
  uint16_t d16;
  while(Size--)
  {
    d16 = LcdRead16();
    *pData = d16;
    pData++;
  }
  LCD_CS_OFF;
  LcdDirWrite();
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadMultiData16to24(uint16_t * pData, uint32_t Size)
{
  uint8_t  rgb888[3];
  while(Size--)
  {
    rgb888[0] = LcdRead8();
    rgb888[1] = LcdRead8();
    rgb888[2] = LcdRead8();
    *pData = (rgb888[0] & 0xF8) << 8 | (rgb888[1] & 0xFC) << 3 | rgb888[2] >> 3;
    pData++;
  }
  LCD_CS_OFF;
  LcdDirWrite();
}

#elif DMANUM(LCD_DMA_RX) > 0 && LCD_SPI > 0

//-----------------------------------------------------------------------------
/* SPI RX on DMA */

//-----------------------------------------------------------------------------
void DMAX_CHANNEL_IRQHANDLER(LCD_DMA_RX)(void)
{
  volatile uint8_t d8 __attribute__((unused));
  if(DMAX(LCD_DMA_RX)->ISR & DMAX_ISR_TCIF(LCD_DMA_RX))
  {
    DMAX(LCD_DMA_RX)->IFCR = DMAX_IFCR_CTCIF(LCD_DMA_RX);
    BITBAND_ACCESS(SPIX->CR2, SPI_CR2_RXDMAEN_Pos) = 0; /* SPI DMA off  */
    while(BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos))
      d8 = *(uint8_t *)&SPIX->DR;
    SPIX->CR1 = (SPIX->CR1 & ~SPI_CR1_BR) | ((LCD_SPI_SPD_READ << SPI_CR1_BR_Pos) | SPI_CR1_BIDIOE);
    LCD_IO_Delay(2 ^ LCD_SPI_SPD_READ);
    while(BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos))
      d8 = SPIX->DR;
    SPIX->CR1 |= SPI_CR1_SPE;
    DMAX_CHANNEL(LCD_DMA_RX)->CCR = 0;
    while(DMAX_CHANNEL(LCD_DMA_RX)->CCR & DMA_CCR_EN);

    #ifndef osFeature_Semaphore
    /* no FreeRtos */
    LCD_IO_DmaTransferStatus = 0;
    #else  // #ifndef osFeature_Semaphore
    /* FreeRtos */
    osSemaphoreRelease(spiDmaBinSemHandle);
    #endif // #else osFeature_Semaphore
  }
  else
    DMAX(LCD_DMA_RX)->IFCR = DMAX_IFCR_CGIF(LCD_DMA_RX);
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadMultiData(void * pData, uint32_t Size, uint32_t dmacr)
{
  DMAX(LCD_DMA_RX)->IFCR = DMAX_IFCR_CGIF(LCD_DMA_RX);
  DMAX_CHANNEL(LCD_DMA_RX)->CCR = 0;  /* DMA stop */
  while(DMAX_CHANNEL(LCD_DMA_RX)->CCR & DMA_CCR_EN);
  DMAX_CHANNEL(LCD_DMA_RX)->CMAR = (uint32_t)pData;  /* memory addr */
  DMAX_CHANNEL(LCD_DMA_RX)->CPAR = (uint32_t)&SPIX->DR; /* periph addr */
  DMAX_CHANNEL(LCD_DMA_RX)->CNDTR = Size;           /* number of data */
  DMAX_CHANNEL(LCD_DMA_RX)->CCR = dmacr;
  DMAX_CHANNEL(LCD_DMA_RX)->CCR |= DMA_CCR_EN;  /* DMA start */
  BITBAND_ACCESS(SPIX->CR2, SPI_CR2_RXDMAEN_Pos) = 1; /* SPI DMA on */
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadMultiData8(uint8_t * pData, uint32_t Size)
{
  uint32_t dmacr;
  dmacr = DMA_CCR_TCIE | (0 << DMA_CCR_MSIZE_Pos) | (0 << DMA_CCR_PSIZE_Pos) |
          DMA_CCR_MINC | (DMAPRIORITY(LCD_DMA_RX) << DMA_CCR_PL_Pos);

  while(Size)
  {
    if(Size > DMA_MAXSIZE)
    {
      LCD_IO_DmaTransferStatus = 2;     /* no last transfer */
      LCD_IO_ReadMultiData((void *)pData, DMA_MAXSIZE, dmacr);
      Size-= DMA_MAXSIZE;
      pData+= DMA_MAXSIZE;
    }
    else
    {
      LCD_IO_DmaTransferStatus = 1;     /* last transfer */
      LCD_IO_ReadMultiData((void *)pData, Size, dmacr);
      Size = 0;
    }
    WaitForDmaEnd();
  }
  LCD_CS_OFF;
  LcdDirWrite();
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadMultiData16(uint16_t * pData, uint32_t Size)
{
  uint32_t dmacr;
  dmacr = DMA_CCR_TCIE | (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos) |
          DMA_CCR_MINC | (DMAPRIORITY(LCD_DMA_RX) << DMA_CCR_PL_Pos);

  while(Size)
  {
    if(Size > DMA_MAXSIZE)
    {
      LCD_IO_DmaTransferStatus = 2;     /* no last transfer */
      LCD_IO_ReadMultiData((void *)pData, DMA_MAXSIZE, dmacr);
      Size-= DMA_MAXSIZE;
      pData+= DMA_MAXSIZE;
    }
    else
    {
      LCD_IO_DmaTransferStatus = 1;     /* last transfer */
      LCD_IO_ReadMultiData((void *)pData, Size, dmacr);
      Size = 0;
    }
    WaitForDmaEnd();
  }
  LCD_CS_OFF;
  LcdDirWrite();
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadMultiData16to24(uint16_t * pData, uint32_t Size)
{
  volatile uint8_t d8 __attribute__((unused));
  uint32_t dmadata_ri = 0, rgb888cnt = 0; /* DMA tempbuffer index, rgb888 index (0..2) */
  uint32_t ntdr_follower;               /* NTDR register folower */
  uint8_t  rgb888[3];
  #if LCD_DMA_RX_BUFMODE == 0
  uint8_t dmadata[LCD_DMA_RX_BUFSIZE] __attribute__((aligned));
  #elif LCD_DMA_RX_BUFMODE == 1
  static uint8_t dmadata[LCD_DMA_RX_BUFSIZE] __attribute__((aligned));
  #elif LCD_DMA_RX_BUFMODE == 2
  uint8_t * dmadata;
  dmadata = LCD_DMA_RX_MALLOC(LCD_DMA_RX_BUFSIZE);
  if(!dmadata)
    return;
  #endif
  DMAX(LCD_DMA_RX)->IFCR = DMAX_IFCR_CGIF(LCD_DMA_RX);
  DMAX_CHANNEL(LCD_DMA_RX)->CCR = 0;  /* DMA stop */
  while(DMAX_CHANNEL(LCD_DMA_RX)->CCR & DMA_CCR_EN);
  DMAX_CHANNEL(LCD_DMA_RX)->CMAR = (uint32_t)dmadata;
  DMAX_CHANNEL(LCD_DMA_RX)->CPAR = (uint32_t)&SPIX->DR;
  DMAX_CHANNEL(LCD_DMA_RX)->CNDTR = LCD_DMA_RX_BUFSIZE;
  ntdr_follower = LCD_DMA_RX_BUFSIZE;
  DMAX_CHANNEL(LCD_DMA_RX)->CCR = (0 << DMA_CCR_MSIZE_Pos) |
      (0 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC |
      (DMAPRIORITY(LCD_DMA_RX) << DMA_CCR_PL_Pos) | DMA_CCR_CIRC;
  DMAX_CHANNEL(LCD_DMA_RX)->CCR |= DMA_CCR_EN;
  BITBAND_ACCESS(SPIX->CR2, SPI_CR2_RXDMAEN_Pos) = 1;
  while(Size)
  {
    if(ntdr_follower != DMAX_CHANNEL(LCD_DMA_RX)->CNDTR)
    {
      if(!--ntdr_follower)
        ntdr_follower = LCD_DMA_RX_BUFSIZE;
      rgb888[rgb888cnt++] = dmadata[dmadata_ri++];
      if(dmadata_ri >= LCD_DMA_RX_BUFSIZE)
        dmadata_ri = 0;
      if(rgb888cnt == 3)
      {
        rgb888cnt = 0;
        Size--;
        *pData++ = (rgb888[0] & 0xF8) << 8 | (rgb888[1] & 0xFC) << 3 | rgb888[2] >> 3;
      }
    }
  }
  BITBAND_ACCESS(SPIX->CR2, SPI_CR2_RXDMAEN_Pos) = 0;
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos))
    d8 = SPIX->DR;
  SPIX->CR1 = (SPIX->CR1 & ~SPI_CR1_BR) | ((LCD_SPI_SPD_WRITE << SPI_CR1_BR_Pos) | SPI_CR1_BIDIOE);
  LCD_IO_Delay(2 ^ LCD_SPI_SPD_READ);
  while(BITBAND_ACCESS(SPIX->SR, SPI_SR_RXNE_Pos))
    d8 = SPIX->DR;
  DMAX_CHANNEL(LCD_DMA_RX)->CCR = 0;
  while(DMAX_CHANNEL(LCD_DMA_RX)->CCR & DMA_CCR_EN);
  LCD_CS_OFF;
  LcdDirWrite();
  #if LCD_DMA_RX_BUFMODE == 2
  LCD_DMA_RX_FREE(dmadata);
  #endif
}

#endif // #elif DMANUM(LCD_DMA_RX) > 0 && LCD_SPI > 0
#endif // #if LCD_SPI_MODE != 0

//=============================================================================
#ifdef  __GNUC__
#pragma GCC push_options
#pragma GCC optimize("O0")
#elif   defined(__CC_ARM)
#pragma push
#pragma O0
#endif
void LCD_IO_Delay(uint32_t c)
{
  while(c--);
}
#ifdef  __GNUC__
#pragma GCC pop_options
#elif   defined(__CC_ARM)
#pragma pop
#endif

//=============================================================================
/* Public functions */

void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

//-----------------------------------------------------------------------------
void LCD_IO_Bl_OnOff(uint8_t Bl)
{
  #if GPIOX_PORTNUM(LCD_BL) >= GPIOX_PORTNUM_A
  if(Bl)
    GPIOX_ODR(LCD_BL) = LCD_BLON;
  else
    GPIOX_ODR(LCD_BL) = 1 - LCD_BLON;
  #endif
}

//-----------------------------------------------------------------------------
void LCD_IO_Init(void)
{
  /* Reset pin clock */
  #if GPIOX_PORTNUM(LCD_RST) >= GPIOX_PORTNUM_A
  #define GPIOX_CLOCK_LCD_RST   GPIOX_CLOCK(LCD_RST)
  #else
  #define GPIOX_CLOCK_LCD_RST   0
  #endif

  /* Backlight pin clock */
  #if GPIOX_PORTNUM(LCD_BL) >= GPIOX_PORTNUM_A
  #define GPIOX_CLOCK_LCD_BL    GPIOX_CLOCK(LCD_BL)
  #else
  #define GPIOX_CLOCK_LCD_BL    0
  #endif

  /* MISO pin clock */
  #if GPIOX_PORTNUM(LCD_MISO) >= GPIOX_PORTNUM_A
  #define GPIOX_CLOCK_LCD_MISO  GPIOX_CLOCK(LCD_MISO)
  #else
  #define GPIOX_CLOCK_LCD_MISO  0
  #endif

  #ifdef  LCD_SPI_ALTERSET
  #define AFIO_CLOCK            RCC_APB2ENR_AFIOEN
  #else
  #define AFIO_CLOCK            0
  #endif

  /* GPIO, DMA Clocks */
  RCC->APB2ENR |= GPIOX_CLOCK(LCD_RS) | GPIOX_CLOCK(LCD_CS) | GPIOX_CLOCK(LCD_SCK) | GPIOX_CLOCK(LCD_MOSI) |
                  GPIOX_CLOCK_LCD_RST | GPIOX_CLOCK_LCD_BL  | GPIOX_CLOCK_LCD_MISO |
		  AFIO_CLOCK;

  /* DMA clock */
  #if LCD_SPI == 0
  #define DMA1_CLOCK_TX         0
  #define DMA1_CLOCK_RX         0
  #else // #if LCD_SPI == 0
  #if DMANUM(LCD_DMA_TX) == 1
  #define DMA1_CLOCK_TX         RCC_AHBENR_DMA1EN
  #elif DMANUM(LCD_DMA_TX) == 2
  #define DMA1_CLOCK_TX         RCC_AHBENR_DMA2EN
  #else
  #define DMA1_CLOCK_TX         0
  #endif
  #if DMANUM(LCD_DMA_RX) == 1
  #define DMA1_CLOCK_RX         RCC_AHBENR_DMA1EN
  #elif DMANUM(LCD_DMA_RX) == 2
  #define DMA1_CLOCK_RX         RCC_AHBENR_DMA2EN
  #else
  #define DMA1_CLOCK_RX         0
  #endif
  #endif  // #else LCD_SPI == 0

  #if (DMANUM(LCD_DMA_TX) >= 1) || (DMANUM(LCD_DMA_RX) >= 1)
  RCC->AHBENR |= DMA1_CLOCK_TX | DMA1_CLOCK_RX;
  #endif

  #ifdef  LCD_SPI_ALTERSET
  LCD_SPI_ALTERSET;
  #endif

  /* MISO = input in full duplex mode */
  #if LCD_SPI_MODE == 2                 // Full duplex
  GPIOX_MODE(MODE_FF_DIGITAL_INPUT, LCD_MISO);
  #endif

  /* Backlight = output, light on */
  #if GPIOX_PORTNUM(LCD_BL) >= GPIOX_PORTNUM_A
  GPIOX_MODE(MODE_PP_OUT_2MHZ, LCD_BL);
  LCD_IO_Bl_OnOff(1);
  #endif

  /* Reset pin = output, reset off */
  #if GPIOX_PORTNUM(LCD_RST) >= GPIOX_PORTNUM_A
  GPIOX_MODE(MODE_PP_OUT_2MHZ, LCD_RST);
  LCD_RST_OFF;
  #endif

  LCD_RS_DATA;
  LCD_CS_OFF;
  GPIOX_MODE(MODE_PP_OUT_50MHZ, LCD_RS);
  GPIOX_MODE(MODE_PP_OUT_50MHZ, LCD_CS);

  GPIOX_ODR(LCD_SCK) = 1;               // SCK = 1

  #if LCD_SPI == 0
  /* Software SPI */
  GPIOX_MODE(MODE_PP_OUT_50MHZ, LCD_SCK);
  GPIOX_MODE(MODE_PP_OUT_50MHZ, LCD_MOSI);

  #else

  /* Hardware SPI */
  LCD_SPI_RCC_EN;

  GPIOX_MODE(MODE_PP_ALTER_50MHZ, LCD_SCK);
  GPIOX_MODE(MODE_PP_ALTER_50MHZ, LCD_MOSI);

  #if LCD_SPI_MODE == 1
  /* Half duplex */
  SPIX->CR1 = SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI | (LCD_SPI_SPD_WRITE << SPI_CR1_BR_Pos) | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
  #else // #if LCD_SPI_MODE == 1
  /* TX or full duplex */
  SPIX->CR1 = SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI | (LCD_SPI_SPD_WRITE << SPI_CR1_BR_Pos);

  #endif // #else LCD_SPI_MODE == 1

  SPIX->CR1 |= SPI_CR1_SPE;

  #endif // #else LCD_SPI == 0

  /* Reset the LCD */
  #if GPIOX_PORTNUM(LCD_RST) >= GPIOX_PORTNUM_A // reset
  LCD_Delay(10);
  LCD_RST_ON;
  LCD_Delay(10);
  LCD_RST_OFF;
  #endif
  LCD_Delay(10);

  #if (DMANUM(LCD_DMA_TX) > 0 || DMANUM(LCD_DMA_RX) > 0) && LCD_SPI > 0
  #ifndef osFeature_Semaphore
  #define DMA_IRQ_PRIORITY    15
  #else
  #define DMA_IRQ_PRIORITY    configLIBRARY_LOWEST_INTERRUPT_PRIORITY
  #endif
  #if DMANUM(LCD_DMA_TX) > 0
  HAL_NVIC_SetPriority(DMAX_CHANNEL_IRQ(LCD_DMA_TX), DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMAX_CHANNEL_IRQ(LCD_DMA_TX));
  #endif
  #if DMANUM(LCD_DMA_RX) > 0
  HAL_NVIC_SetPriority(DMAX_CHANNEL_IRQ(LCD_DMA_RX), DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMAX_CHANNEL_IRQ(LCD_DMA_RX));
  #endif
  #ifdef osFeature_Semaphore
  osSemaphoreDef(spiDmaBinSem);
  spiDmaBinSemHandle = osSemaphoreCreate(osSemaphore(spiDmaBinSem), 1);
  osSemaphoreWait(spiDmaBinSemHandle, 1);
  #endif
  #endif  // #if DMANUM(LCD_DMA_RX) > 0
} // void LCD_IO_Init(void)

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd8(uint8_t Cmd)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdCmdWrite8(Cmd);
  LCD_CS_OFF;
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd16(uint16_t Cmd)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdCmdWrite16(Cmd);
  LCD_CS_OFF;
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteData8(uint8_t Data)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdWrite8(Data);
  LCD_CS_OFF;
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteData16(uint16_t Data)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdWrite16(Data);
  LCD_CS_OFF;
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd8DataFill16(uint8_t Cmd, uint16_t Data, uint32_t Size)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdCmdWrite8(Cmd);
  LcdSpiMode16();
  LCD_IO_WriteMultiData16(&Data, Size, 0);
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd8MultipleData8(uint8_t Cmd, uint8_t *pData, uint32_t Size)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdCmdWrite8(Cmd);
  LCD_IO_WriteMultiData8(pData, Size, 1);
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd8MultipleData16(uint8_t Cmd, uint16_t *pData, uint32_t Size)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdCmdWrite8(Cmd);
  LcdSpiMode16();
  LCD_IO_WriteMultiData16(pData, Size, 1);
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd16DataFill16(uint16_t Cmd, uint16_t Data, uint32_t Size)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdCmdWrite16(Cmd);
  LCD_IO_WriteMultiData16(&Data, Size, 0);
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd16MultipleData8(uint16_t Cmd, uint8_t *pData, uint32_t Size)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdCmdWrite16(Cmd);
  LcdSpiMode8();
  LCD_IO_WriteMultiData8(pData, Size, 1);
}

//-----------------------------------------------------------------------------
void LCD_IO_WriteCmd16MultipleData16(uint16_t Cmd, uint16_t *pData, uint32_t Size)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdCmdWrite16(Cmd);
  LCD_IO_WriteMultiData16(pData, Size, 1);
}

#if LCD_SPI_MODE == 0
__weak void LCD_IO_ReadCmd8MultipleData8(uint8_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize) {}
__weak void LCD_IO_ReadCmd8MultipleData16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize) {}
__weak void LCD_IO_ReadCmd8MultipleData24to16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize) {}
__weak void LCD_IO_ReadCmd16MultipleData8(uint16_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize) {}
__weak void LCD_IO_ReadCmd16MultipleData16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize) {}
__weak void LCD_IO_ReadCmd16MultipleData24to16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize) {}
#else

//-----------------------------------------------------------------------------
void LCD_IO_ReadCmd8MultipleData8(uint8_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdCmdWrite8(Cmd);

  DummySize = (DummySize << 3) + LCD_SCK_EXTRACLK;
  LcdDirRead(DummySize);
  LCD_IO_ReadMultiData8(pData, Size);
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadCmd8MultipleData16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdCmdWrite8(Cmd);
  LcdSpiMode16();

  DummySize = (DummySize << 3) + LCD_SCK_EXTRACLK;
  LcdDirRead(DummySize);
  LCD_IO_ReadMultiData16(pData, Size);
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadCmd8MultipleData24to16(uint8_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize)
{
  WaitForDmaEnd();
  LcdSpiMode8();
  LCD_CS_ON;
  LcdCmdWrite8(Cmd);

  DummySize = (DummySize << 3) + LCD_SCK_EXTRACLK;
  LcdDirRead(DummySize);
  LCD_IO_ReadMultiData16to24(pData, Size);
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadCmd16MultipleData8(uint16_t Cmd, uint8_t *pData, uint32_t Size, uint32_t DummySize)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdCmdWrite16(Cmd);
  LcdSpiMode8();

  DummySize = (DummySize << 3) + LCD_SCK_EXTRACLK;
  LcdDirRead(DummySize);
  LCD_IO_ReadMultiData8(pData, Size);
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadCmd16MultipleData16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdCmdWrite16(Cmd);

  DummySize = (DummySize << 3) + LCD_SCK_EXTRACLK;
  LcdDirRead(DummySize);
  LCD_IO_ReadMultiData16(pData, Size);
}

//-----------------------------------------------------------------------------
void LCD_IO_ReadCmd16MultipleData24to16(uint16_t Cmd, uint16_t *pData, uint32_t Size, uint32_t DummySize)
{
  WaitForDmaEnd();
  LcdSpiMode16();
  LCD_CS_ON;
  LcdCmdWrite16(Cmd);
  LcdSpiMode8();

  DummySize = (DummySize << 3) + LCD_SCK_EXTRACLK;
  LcdDirRead(DummySize);
  LCD_IO_ReadMultiData16to24(pData, Size);
}

#endif // #else LCD_SPI_MODE == 0
