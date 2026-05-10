/**********************************************
LiquidCrystal_I2C
last updated on 21/12/2011
Tim Starling Fix the reset bug (Thanks Tim)
www.yfrobot.com
**********************************************/

#ifndef LiquidCrystal_I2C_h
#define LiquidCrystal_I2C_h

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit

// LCD structure
typedef struct {
  uint8_t _Addr;
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _numlines;
  uint8_t _cols;
  uint8_t _rows;
  uint8_t _backlightval;
} LiquidCrystal_I2C;

// Function prototypes
void LCD_Init(LiquidCrystal_I2C* lcd, uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows);
void LCD_Begin(LiquidCrystal_I2C* lcd, uint8_t cols, uint8_t rows, uint8_t charsize);
void LCD_Clear(LiquidCrystal_I2C* lcd);
void LCD_Home(LiquidCrystal_I2C* lcd);
void LCD_NoDisplay(LiquidCrystal_I2C* lcd);
void LCD_Display(LiquidCrystal_I2C* lcd);
void LCD_NoBlink(LiquidCrystal_I2C* lcd);
void LCD_Blink(LiquidCrystal_I2C* lcd);
void LCD_NoCursor(LiquidCrystal_I2C* lcd);
void LCD_Cursor(LiquidCrystal_I2C* lcd);
void LCD_ScrollDisplayLeft(LiquidCrystal_I2C* lcd);
void LCD_ScrollDisplayRight(LiquidCrystal_I2C* lcd);
void LCD_LeftToRight(LiquidCrystal_I2C* lcd);
void LCD_RightToLeft(LiquidCrystal_I2C* lcd);
void LCD_Autoscroll(LiquidCrystal_I2C* lcd);
void LCD_NoAutoscroll(LiquidCrystal_I2C* lcd);
void LCD_CreateChar(LiquidCrystal_I2C* lcd, uint8_t location, uint8_t charmap[]);
void LCD_SetCursor(LiquidCrystal_I2C* lcd, uint8_t col, uint8_t row);
void LCD_Write(LiquidCrystal_I2C* lcd, uint8_t value);
void LCD_Command(LiquidCrystal_I2C* lcd, uint8_t value);
void LCD_Print(LiquidCrystal_I2C* lcd, const char *str);
void LCD_NoBacklight(LiquidCrystal_I2C* lcd);
void LCD_Backlight(LiquidCrystal_I2C* lcd);
void LCD_SetBacklight(LiquidCrystal_I2C* lcd, uint8_t new_val);
void LCD_PrintStr(LiquidCrystal_I2C* lcd, const char *c);

#ifdef __cplusplus
}
#endif

#endif