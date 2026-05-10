/**********************************************
LiquidCrystal_I2C
last updated on 21/12/2011
Tim Starling Fix the reset bug (Thanks Tim)
www.yfrobot.com
**********************************************/


#include "LiquidCrystal_I2C.h"
#include <string.h>
#include "i2c.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

static void send(LiquidCrystal_I2C* lcd, uint8_t value, uint8_t mode);
static void write4bits(LiquidCrystal_I2C* lcd, uint8_t value);
static void expanderWrite(LiquidCrystal_I2C* lcd, uint8_t _data);
static void pulseEnable(LiquidCrystal_I2C* lcd, uint8_t _data);
static void init_priv(LiquidCrystal_I2C* lcd);

void LCD_Init(LiquidCrystal_I2C* lcd, uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows)
{
  lcd->_Addr = lcd_Addr;
  lcd->_cols = lcd_cols;
  lcd->_rows = lcd_rows;
  lcd->_backlightval = LCD_NOBACKLIGHT;
  init_priv(lcd);
}

void init_priv(LiquidCrystal_I2C* lcd)
{
  lcd->_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  LCD_Begin(lcd, lcd->_cols, lcd->_rows, LCD_5x8DOTS);  
}

void LCD_Begin(LiquidCrystal_I2C* lcd, uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    lcd->_displayfunction |= LCD_2LINE;
  }
  lcd->_numlines = lines;

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (lines == 1)) {
    lcd->_displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  HAL_Delay(50); 
  
  // Now we pull both RS and R/W low to begin commands
  expanderWrite(lcd, lcd->_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
  HAL_Delay(1000);

  	//put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46
  
  	  // we start in 8bit mode, try to set 4 bit mode
   write4bits(lcd, 0x03 << 4);
   HAL_Delay(5); // wait min 4.1ms
   
   // second try
   write4bits(lcd, 0x03 << 4);
   HAL_Delay(5); // wait min 4.1ms
   
   // third go!
   write4bits(lcd, 0x03 << 4); 
   HAL_Delay(1);
   
   // finally, set to 4-bit interface
   write4bits(lcd, 0x02 << 4); 


  // set # lines, font size, etc.
  LCD_Command(lcd, LCD_FUNCTIONSET | lcd->_displayfunction);  
  
  // turn the display on with no cursor or blinking default
  lcd->_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  LCD_Display(lcd);
  
  // clear it off
  LCD_Clear(lcd);
  
  // Initialize to default text direction (for roman languages)
  lcd->_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  
  // set the entry mode
  LCD_Command(lcd, LCD_ENTRYMODESET | lcd->_displaymode);
  
  LCD_Home(lcd);
  
}

void LCD_Clear(LiquidCrystal_I2C* lcd){
  LCD_Command(lcd, LCD_CLEARDISPLAY);// clear display, set cursor position to zero
  HAL_Delay(2);  // this command takes a long time!
}

void LCD_Home(LiquidCrystal_I2C* lcd){
  LCD_Command(lcd, LCD_RETURNHOME);  // set cursor position to zero
  HAL_Delay(2);  // this command takes a long time!
}

void LCD_SetCursor(LiquidCrystal_I2C* lcd, uint8_t col, uint8_t row){
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > lcd->_numlines ) {
    row = lcd->_numlines-1;    // we count rows starting w/0
  }
  LCD_Command(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCD_NoDisplay(LiquidCrystal_I2C* lcd) {
  lcd->_displaycontrol &= ~LCD_DISPLAYON;
  LCD_Command(lcd, LCD_DISPLAYCONTROL | lcd->_displaycontrol);
}

void LCD_Display(LiquidCrystal_I2C* lcd) {
  lcd->_displaycontrol |= LCD_DISPLAYON;
  LCD_Command(lcd, LCD_DISPLAYCONTROL | lcd->_displaycontrol);
}

void LCD_NoCursor(LiquidCrystal_I2C* lcd) {
  lcd->_displaycontrol &= ~LCD_CURSORON;
  LCD_Command(lcd, LCD_DISPLAYCONTROL | lcd->_displaycontrol);
}

void LCD_Cursor(LiquidCrystal_I2C* lcd) {
  lcd->_displaycontrol |= LCD_CURSORON;
  LCD_Command(lcd, LCD_DISPLAYCONTROL | lcd->_displaycontrol);
}

void LCD_NoBlink(LiquidCrystal_I2C* lcd) {
  lcd->_displaycontrol &= ~LCD_BLINKON;
  LCD_Command(lcd, LCD_DISPLAYCONTROL | lcd->_displaycontrol);
}

void LCD_Blink(LiquidCrystal_I2C* lcd) {
  lcd->_displaycontrol |= LCD_BLINKON;
  LCD_Command(lcd, LCD_DISPLAYCONTROL | lcd->_displaycontrol);
}

void LCD_ScrollDisplayLeft(LiquidCrystal_I2C* lcd) {
  LCD_Command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LCD_ScrollDisplayRight(LiquidCrystal_I2C* lcd) {
  LCD_Command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void LCD_LeftToRight(LiquidCrystal_I2C* lcd) {
  lcd->_displaymode |= LCD_ENTRYLEFT;
  LCD_Command(lcd, LCD_ENTRYMODESET | lcd->_displaymode);
}

void LCD_RightToLeft(LiquidCrystal_I2C* lcd) {
  lcd->_displaymode &= ~LCD_ENTRYLEFT;
  LCD_Command(lcd, LCD_ENTRYMODESET | lcd->_displaymode);
}

void LCD_Autoscroll(LiquidCrystal_I2C* lcd) {
  lcd->_displaymode |= LCD_ENTRYSHIFTINCREMENT;
  LCD_Command(lcd, LCD_ENTRYMODESET | lcd->_displaymode);
}

void LCD_NoAutoscroll(LiquidCrystal_I2C* lcd) {
  lcd->_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  LCD_Command(lcd, LCD_ENTRYMODESET | lcd->_displaymode);
}

void LCD_CreateChar(LiquidCrystal_I2C* lcd, uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  LCD_Command(lcd, LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    LCD_Write(lcd, charmap[i]);
  }
}

void LCD_NoBacklight(LiquidCrystal_I2C* lcd) {
  lcd->_backlightval=LCD_NOBACKLIGHT;
  expanderWrite(lcd, 0);
}

void LCD_Backlight(LiquidCrystal_I2C* lcd) {
  lcd->_backlightval=LCD_BACKLIGHT;
  expanderWrite(lcd, 0);
}

void LCD_SetBacklight(LiquidCrystal_I2C* lcd, uint8_t new_val){
  if(new_val){
    LCD_Backlight(lcd);		// turn backlight on
  }else{
    LCD_NoBacklight(lcd);		// turn backlight off
  }
}

void LCD_Command(LiquidCrystal_I2C* lcd, uint8_t value) {
  send(lcd, value, 0);
}

void send(LiquidCrystal_I2C* lcd, uint8_t value, uint8_t mode) {
  uint8_t highnib=value&0xf0;
  uint8_t lownib=(value<<4)&0xf0;
  write4bits(lcd, (highnib)|mode);
  write4bits(lcd, (lownib)|mode); 
}

void write4bits(LiquidCrystal_I2C* lcd, uint8_t value) {
  expanderWrite(lcd, value);
  pulseEnable(lcd, value);
}

void expanderWrite(LiquidCrystal_I2C* lcd, uint8_t _data){                                        
  uint8_t data = (uint8_t)(_data) | lcd->_backlightval;
  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(lcd->_Addr << 1), &data, 1, HAL_MAX_DELAY);
}

void pulseEnable(LiquidCrystal_I2C* lcd, uint8_t _data){
  expanderWrite(lcd, _data | En);	// En high
  HAL_Delay(1);		// enable pulse must be >450ns
  
  expanderWrite(lcd, _data & ~En);	// En low
  HAL_Delay(1);		// commands need > 37us to settle
}

void LCD_Write(LiquidCrystal_I2C* lcd, uint8_t value) {
  send(lcd, value, Rs);
}

void LCD_Print(LiquidCrystal_I2C* lcd, const char *str) {
  while (*str) {
    LCD_Write(lcd, *str++);
  }
}

void LCD_PrintStr(LiquidCrystal_I2C* lcd, const char *c){
  LCD_Print(lcd, c);
}