
#include <Arduino.h>

// Define pins for sensors and buzzer
#define EMIT_PIN 11
#define BUZZER_PIN A7

// Include necessary libraries for line sensing, motor control, kinematics, PID and encoders
#include "linesensor.h"
#include "motors.h"
#include "kinematics.h"
#include "pid.h"
#include "encoders.h"
#include "LCD_HD44780.h"

// Declare instances of necessary classes
LineSensor_c line_sensors;
Motors_c motors(10, 16, 9, 15);  // Motor control with specified pins
Kinematics_c kinematics;
LCD_HD44780 lcd; // LCD class

// Define constants for line sensing thresholds and motor control
const int ACTIVE_THRESHOLD = 2150;
const int INACTIVE2 = 1200;
const int INACTIVE_THRESHOLD = 800;
const int BiasPWM = 25;  // Base motor power for line following
const int MaxTurnPWM = 26;  // Maximum turn power

// Declare global variables for timers and previous states
unsigned long DN3InactiveStartTime = 0;
unsigned long startup_time;
unsigned long previous_time = 0;
unsigned long gapStartTime;
float previous_count_Left = 0;
float previous_count_Right = 0;

// Boolean flags for robot states
bool onLine = false;
bool DN3InactiveTimerStarted = false;
bool canTurn180 = true;
bool gapTimerStarted = false;

int turn_dir;  // Direction to turn: -1 for left, 1 for right

// Define states for the robot's behavior
enum RobotState {
  STARTUP,
  FINDING_LINE_WITH_DN3,
  ALIGNING_TO_LINE,
  LINE_FOLLOWING,
  GAP_CROSSING
};

// Initial state of the robot
RobotState currentState = STARTUP;

void setup() {
  startup_time = millis();  // Record the start time for later use
  pinMode(BUZZER_PIN, OUTPUT);  // Set buzzer pin as output
  setupEncoder0();  // Initialize encoder 0
  setupEncoder1();  // Initialize encoder 1
  line_sensors.initialize();  // Initialize line sensors
  motors.initialize();  // Initialize motors
  Serial.begin(9600);  // Start serial communication for debugging
  Serial.println("Initializing LCD...");
  lcd.init();
  Serial.println("LCD Initialized.");
}

// Function to make a beep sound
void beep() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

// Function to check if a line is detected
int lineDetected() {
  float left_sensor = line_sensors.readLineSensor(DN2);
  float middle_sensor = line_sensors.readLineSensor(DN3);
  float right_sensor = line_sensors.readLineSensor(DN4);

  if (middle_sensor >= ACTIVE_THRESHOLD) {
    return 1;
  } else {
    return -1;
  }
}

// Function to turn the robot until it detects a line
void turnUntilLine() {
  float readDN1 = line_sensors.readLineSensor(DN1);
  float readDN3 = line_sensors.readLineSensor(DN3);
  float readDN5 = line_sensors.readLineSensor(DN5);
  if (readDN3 > 1800) {
    motors.setMotorPower(0, 0);  // Stop if middle sensor detects line
  } else {
    if (turn_dir < 0) {  // Turn left
      motors.setMotorPower(-50, 45);
    } else {  // Turn right
      motors.setMotorPower(45, -45);
    }
  }
}

// Calculate a weighted measurement for line following
float weightedMeasurement() {
  float readDN2 = line_sensors.readLineSensor(DN2);
  float readDN4 = line_sensors.readLineSensor(DN4);
  float Sum = readDN2 + readDN4;  // Calculate sum of left and right sensors
  float N2 = readDN2 / Sum;  // Normalize left sensor reading
  float N4 = readDN4 / Sum;  // Normalize right sensor reading

  N2 *= 2.0;
  N4 *= 2.0;
  return N2 - N4;
}

// Function to make robot turn 180 degrees
void turn180Degree() {
  if (!canTurn180) {
    return;
  }
  motors.setMotorPower(0, 0);  // Stop motors
  delay(300);
  motors.setMotorPower(29, -29);  // Turn around
  delay(1000);
  currentState = FINDING_LINE_WITH_DN3;
}

// This function implements the line-following logic.
void followLine() {
  // Read sensor values for DN1 to DN5
  float readDN1 = line_sensors.readLineSensor(DN1);
  float readDN2 = line_sensors.readLineSensor(DN2);
  float readDN3 = line_sensors.readLineSensor(DN3);
  float readDN4 = line_sensors.readLineSensor(DN4);
  float readDN5 = line_sensors.readLineSensor(DN5);

  // Check if the line is detected under the middle sensor
  int detectedSensor = lineDetected();

  // Update onLine variable based on sensor readings
  if (detectedSensor != 1) {
    onLine = true;
  } else if (readDN2 >= ACTIVE_THRESHOLD || readDN4 >= ACTIVE_THRESHOLD) {
    onLine = true;
  } else {
    onLine = false;
  }

  // If the line is detected under any sensor, calculate the motor powers
  if (onLine) {
    // Calculate motor powers based on line position
    float W = weightedMeasurement();
    float leftPWM = BiasPWM - (W * MaxTurnPWM);
    float rightPWM = BiasPWM + (W * MaxTurnPWM);

    motors.setMotorPower(leftPWM, rightPWM);

  } else {
    // Stop motors if line is not detected
    motors.setMotorPower(0, 0);
  }

  // Check for inactivity of the DN3 sensor and decide if a 180-degree turn is required
  if (currentState != STARTUP) {
    if (line_sensors.readLineSensor(DN3) < INACTIVE_THRESHOLD) {
      if (!DN3InactiveTimerStarted) {
        DN3InactiveStartTime = millis();
        DN3InactiveTimerStarted = true;
      }
      if (millis() - DN3InactiveStartTime > 1000) {
        DN3InactiveTimerStarted = false;
        turn180Degree();
      }
    } else {
      DN3InactiveTimerStarted = false;
    }
  }
}

// MAIN LOOP
void loop() {
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hello, World!");
  Serial.println("Message should be displayed on LCD now.");
  delay(2000);


  // if (millis() - startup_time > 11000 && canTurn180) {
  //   canTurn180 = false;
  // }

  // unsigned long current_time = micros();
  // float delta_t = (current_time - previous_time) / 1000000.0;
  
  // // Update Kinematics based on encoders readings
  // kinematics.update(count_Left, count_Right, delta_t);
  // //Serial.print("X Position: ");
  // //Serial.println(kinematics.getX());
  // //Serial.print("Y Position: ");
  // //Serial.println(kinematics.getY());
  // //Serial.print("Theta (radians): ");
  // //Serial.println(kinematics.getTheta());
  // //Serial.print("delta_t: ");
  // //Serial.println(delta_t);

  // //Serial.print("Encoder Value 0: ");
  // //Serial.println(count_Right);
  // //Serial.print("Encoder Value 1: ");
  // //Serial.println(count_Left);
  
  // // Get the turning direction based on DN1 and DN5 sensor readings
  // turn_dir = 0;
  // float readDN1 = line_sensors.readLineSensor(DN1);
  // float readDN5 = line_sensors.readLineSensor(DN5);

  // if (readDN1 > 1200) turn_dir = -1;
  // if (readDN5 > 1200) turn_dir = 1;

  //  // Turn until a line is detected or continue following the line
  // if (turn_dir != 0) {
  //   turnUntilLine();
  // } else {
  //   followLine();
  // }

  //  // State machine logic to decide the robot's actions based on current state
  // switch (currentState) {
  //   case STARTUP:
  //     // For 2.5 seconds after startup, move forward slowly
  //     if (millis() - startup_time < 2500) {
  //       motors.setMotorPower(20, 23);
  //     } else {
  //       // Transition to finding the line with DN3
  //       currentState = FINDING_LINE_WITH_DN3;
  //       beep();
  //     }
  //     break;

  //   case FINDING_LINE_WITH_DN3:
  //     if (line_sensors.readLineSensor(DN3) >= ACTIVE_THRESHOLD) {
  //       motors.setMotorPower(20, 20);
  //       if (line_sensors.readLineSensor(DN1) >= ACTIVE_THRESHOLD ||
  //           line_sensors.readLineSensor(DN5) >= ACTIVE_THRESHOLD) {
  //         currentState = ALIGNING_TO_LINE;
  //         beep();
  //       }
  //     }
  //     break;

  //   case ALIGNING_TO_LINE:
  //     if (millis() - startup_time < 100) return;
  //     motors.setMotorPower(0, 0);
  //     if (line_sensors.readLineSensor(DN1) >= ACTIVE_THRESHOLD) {
  //       turn_dir = -1;
  //     } else {
  //       turn_dir = 1;
  //     }
  //     turnUntilLine();
  //     currentState = LINE_FOLLOWING;
  //     beep();
  //     break;

  //   case LINE_FOLLOWING:
  //     followLine();

  //     // Check if DN2, DN3, and DN4 do not sense a line for 500ms
  //     if (line_sensors.readLineSensor(DN2) < INACTIVE2 &&
  //         line_sensors.readLineSensor(DN3) < INACTIVE2 &&
  //         line_sensors.readLineSensor(DN4) < INACTIVE2) {

  //       if (!gapTimerStarted) {
  //         gapStartTime = millis();
  //         gapTimerStarted = true;
  //       }

  //       if (millis() - gapStartTime > 50) {
  //         //motors.setMotorPower(0, 0);
  //         currentState = GAP_CROSSING;
  //         beep();
  //       } else {
  //         gapTimerStarted = false;
  //       }
  //     }
  //     break;
  //   case GAP_CROSSING:
  //     motors.setMotorPower(20, 20);
  //     if (line_sensors.readLineSensor(DN1) >= ACTIVE_THRESHOLD ||
  //         line_sensors.readLineSensor(DN2) >= ACTIVE_THRESHOLD ||
  //         line_sensors.readLineSensor(DN3) >= ACTIVE_THRESHOLD ||
  //         line_sensors.readLineSensor(DN4) >= ACTIVE_THRESHOLD ||
  //         line_sensors.readLineSensor(DN5) >= ACTIVE_THRESHOLD) {
        
  //       currentState = LINE_FOLLOWING;
  //       beep();
  //     }
  //     break;
  // }
  // // Save the current encoder readings for the next loop iteration
  // previous_count_Left = count_Left;
  // previous_count_Right = count_Right;
}