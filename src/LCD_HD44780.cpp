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
  int row_offsets[] = { 0x00, 0x40 };
  command(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

void LCD_HD44780::print(const String &message) {
  for (int i = 0; i < message.length(); i++) {
    write(message[i]);
  }
}

void LCD_HD44780::command(uint8_t value) {
  send(value, LOW);
}

void LCD_HD44780::write(uint8_t value) {
  send(value, HIGH);
}

void LCD_HD44780::pulseEnable() {
  digitalWrite(ENABLE_PIN, LOW);
  delayMicroseconds(1);    
  digitalWrite(ENABLE_PIN, HIGH);
  delayMicroseconds(1);    // enable pulse must be >450ns
  digitalWrite(ENABLE_PIN, LOW);
  delayMicroseconds(100);   // commands need > 37us to settle
}

void LCD_HD44780::write4bits(uint8_t value) {
  digitalWrite(D4_PIN, (value >> 0) & 0x01);
  digitalWrite(D5_PIN, (value >> 1) & 0x01);
  digitalWrite(D6_PIN, (value >> 2) & 0x01);
  digitalWrite(D7_PIN, (value >> 3) & 0x01);

  pulseEnable();
}

void LCD_HD44780::send(uint8_t value, uint8_t mode) {
  digitalWrite(RS_PIN, mode);

  // Write the upper 4 bits
  write4bits(value >> 4);
  // Write the lower 4 bits
  write4bits(value);
}

// Now you can use this class in your main code to control the LCD.
// For example:
// LCD_HD44780 lcd;
// lcd.init();
// lcd.setCursor(0, 0);
// lcd.print("Hello, World!");
