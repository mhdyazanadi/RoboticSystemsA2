#ifndef LCD_HD44780_H
#define LCD_HD44780_H

#include <Arduino.h>

// HD44780 LCD controller commands
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

// flags for display entry mode
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAY_MOVE 0x08
#define LCD_CURSOR_MOVE 0x00
#define LCD_MOVE_RIGHT 0x04
#define LCD_MOVE_LEFT 0x00

// flags for function set
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define RS_PIN 0
#define ENABLE_PIN 1
#define D4_PIN 14
#define D5_PIN 17
#define D6_PIN 13
#define D7_PIN 30

class LCD_HD44780 {
public:
    LCD_HD44780();

    void init();
    void clear();
    void home();
    void setCursor(uint8_t col, uint8_t row);
    void print(const String &message);

private:
    void command(uint8_t value);
    void write(uint8_t value);
    void pulseEnable();
    void write4bits(uint8_t value);
    void send(uint8_t value, uint8_t mode);
};

#endif // LCD_HD44780_H
