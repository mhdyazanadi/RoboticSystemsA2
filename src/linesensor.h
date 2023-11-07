#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#include <Arduino.h>

// Define line sensor pins
#define LS_LEFT_PIN 12
#define LS_MIDLEFT_PIN A0
#define LS_MIDDLE_PIN A2
#define LS_MIDRIGHT_PIN A3
#define LS_RIGHT_PIN A4

// Defining these constants so they can be called
// in a readable format in main adruino sketch
const int DN1 = 0;
const int DN2 = 1;
const int DN3 = 2;
const int DN4 = 3;
const int DN5 = 4;

class LineSensor_c {
public:
  
  // Constructor, IS IT?, though
  LineSensor_c() {}

  // Initialize line sensor pins and other settings
  void initialize() {
    pinMode(EMIT_PIN, INPUT); // Set EMIT as an input (off)
    for (int i = 0; i < 5; i++) {
      pinMode(ls_pins[i], INPUT); // Set line sensor pins to input
    }
  }
  // Read a specific line sensor (number should be between 0 and 4)
  float readLineSensor(int number) {
    if (number < 0 || number > 4) {
      return -1.0; // Return a negative value to indicate an error
    }

    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);

    pinMode(ls_pins[number], OUTPUT);
    digitalWrite(ls_pins[number], HIGH);
    delayMicroseconds(10);

    // Set all line sensor pins to input
    for (int i = 0; i < 5; i++) {
      pinMode(ls_pins[i], INPUT);
    }

    unsigned long start_time = micros();
    unsigned long max_duration = 5000; // Set a maximum duration for reading

    while (digitalRead(ls_pins[number]) == HIGH) {
      if (micros() - start_time > max_duration) {
        // Finish the sensor reading if the time-out occurs
        break;
      }
    }

    unsigned long end_time = micros();

    pinMode(EMIT_PIN, INPUT);

    unsigned long elapsed_time = end_time - start_time;

    return (float)elapsed_time;
  }

private:
  // Store our pin numbers into an array
  int ls_pins[5] = {LS_LEFT_PIN, 
                    LS_MIDLEFT_PIN, 
                    LS_MIDDLE_PIN, 
                    LS_MIDRIGHT_PIN, 
                    LS_RIGHT_PIN};
};
#endif
