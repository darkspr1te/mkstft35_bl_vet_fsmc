// 3D Filled Vector Graphics
// (c) 2019 Pawel A. Hernik
// YouTube videos:
// https://youtu.be/YLf2WXjunyg
// https://youtu.be/5y28ipwQs-E

/* Screen dimension (width / height) */
//#define  ILI9488_LCD_PIXEL_WIDTH   320
//#define  ILI9488_LCD_PIXEL_HEIGHT  480
#define SCR_WD  320
#define SCR_HT  480

/* 3d field dimension (width / height) */
#define WD_3D   256
#define HT_3D   256

/* Statistic character size */
#define CHARSIZEX 6
#define CHARSIZEY 8

/* Frame Buffer lines */
#define NLINES   16

/* Double buffer (if DMA transfer used -> speed inc)
   - 0 Double buffer disabled
   - 1 Double buffer enabled */
#define DOUBLEBUF 0

/* Button pin assign */
#define BUTTON    C, 5    /* If not used leave it that way */
/* Button active level (0 or 1) */
#define BUTTON_ON    1
/* Button input mode (0 = pull none, 1 = pull up, 2 = pull down) */
#define BUTTON_PU    0
