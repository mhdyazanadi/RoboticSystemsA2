#include <Arduino.h>
#ifndef _MOTORS_H
#define _MOTORS_H







#define FWD LOW  // Go forward
#define REV HIGH  // Go backward
class Motors_c {
public:
  Motors_c(int leftPWM, int leftDir, int rightPWM, int rightDir) : L_PWM_PIN(leftPWM), L_DIR_PIN(leftDir), R_PWM_PIN(rightPWM), R_DIR_PIN(rightDir) {}
// JUST A COMMMENT
  void initialize() {
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);

    digitalWrite(L_DIR_PIN, FWD); // Set left motor direction to forward
    digitalWrite(R_DIR_PIN, FWD); // Set right motor direction to forward

    analogWrite(L_PWM_PIN, 0); // Set left motor power to 0
    analogWrite(R_PWM_PIN, 0); // Set right motor power to 0
  }

  void setMotorPower(float left_pwm, float right_pwm) {
    if (left_pwm < -255) {
      left_pwm = -255;
    } else if (left_pwm > 255) {
      left_pwm = 255;
    }

    if (right_pwm < -255) {
      right_pwm = -255;
    } else if (right_pwm > 255) {
      right_pwm = 255;
    }

    if (left_pwm < 0) {
      digitalWrite(L_DIR_PIN, REV); // Set the left motor to reverse
      analogWrite(L_PWM_PIN, -left_pwm);
    } else {
      digitalWrite(L_DIR_PIN, FWD); // Set the left motor to forward
      analogWrite(L_PWM_PIN, left_pwm);
    }

    if (right_pwm < 0) {
      digitalWrite(R_DIR_PIN, REV); // Set the right motor to reverse
      analogWrite(R_PWM_PIN, -right_pwm);
    } else {
      digitalWrite(R_DIR_PIN, FWD); // Set the right motor to forward
      analogWrite(R_PWM_PIN, right_pwm);
    }
  }

  void rotateInPlace(float rotationalPower) {
    // Rotate in place by applying equal rotational power to both motors
    setMotorPower(rotationalPower, -rotationalPower);
  }

private:
  int L_PWM_PIN;
  int L_DIR_PIN;
  int R_PWM_PIN;
  int R_DIR_PIN;
};

#endif
