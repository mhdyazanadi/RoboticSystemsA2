#include "LCD_HD44780.h"

LCD_HD44780::LCD_HD44780() {
  // Constructor is left empty
}

void LCD_HD44780::init() {
  // Set up the pins to be outputs
  pinMode(RS_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(D4_PIN, OUTPUT);
  pinMode(D5_PIN, OUTPUT);
  pinMode(D6_PIN, OUTPUT);
  pinMode(D7_PIN, OUTPUT);

  // Wait for LCD to become ready (takes a little while)
  delayMicroseconds(50000);

  // We start in 8bit mode, try to set to 4 bit mode
  write4bits(0x03);
  delayMicroseconds(4500); // wait min 4.1ms

  // Second try
  write4bits(0x03);
  delayMicroseconds(4500); // wait min 4.1ms

  // Third go!
  write4bits(0x03); 
  delayMicroseconds(150);

  // Finally, set to 4-bit interface
  write4bits(0x02); 

  // Set # lines, font size, etc.
  command(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8DOTS);
  
  // Turn the display on with no cursor or blinking default
  command(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);

  // Clear it off
  clear();

  // Set the entry mode
  command(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
}

void LCD_HD44780::clear() {
  command(LCD_CLEAR_DISPLAY);
  delayMicroseconds(2000);  // this command takes a long time!
}

void LCD_HD44780::home() {
  command(LCD_RETURN_HOME);
  delayMicroseconds(2000);  // this command takes a long time!
}

void LCD_HD44780::setCursor(uint8_t col, uint8_t row) {
    int row_offsets[] = { 0x00, 0x40, 0x14 };
    if (row > rows()) return;
    command(col + row_offsets[row]);
}
