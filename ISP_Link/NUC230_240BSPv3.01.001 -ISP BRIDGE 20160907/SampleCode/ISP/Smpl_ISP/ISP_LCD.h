#define SPI_LCD_PORT  SPI3
#define SPI_CS_CLR    SPI3->SSR=1
#define SPI_CS_SET    SPI3->SSR=0
#define LCM_DC_SET   	PD12 = 1
#define LCM_DC_CLR    PD12 = 0

#define LCM_RESET_SET PD13 = 1
#define LCM_RESET_CLR PD13 = 0

#define LCM_LED_SET   PB4 = 1
#define LCM_LED_CLR   PB4 = 0

extern uint8_t Font8x16[];

#define	White           0xFFFF
#define Black           0x0000
#define Blue            0x001F
#define Blue2           0x051F
#define Red             0xF800
#define Magenta         0xF81F
#define Green           0x07E0
#define Cyan            0x7FFF
#define Yellow          0xFFE0
extern void LCD_initial(void);
extern void ILI9341_LCD_PutString(uint16_t x, uint16_t y,uint8_t *s, uint32_t fColor, uint32_t bColor);
extern void ILI9341_LCD_PutString_line(uint16_t x, uint16_t y,uint8_t *s, uint32_t fColor, uint32_t bColor);
extern void dsp_single_color(unsigned int LCD_DATA);
extern void LCD_Fill(unsigned int x_start, unsigned int y_start,unsigned int x_end, unsigned int y_end,  unsigned int LCD_DATA);
