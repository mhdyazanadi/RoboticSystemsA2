// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H



// This class is for motors operation in 3pi only.
// It defines the pins, direction, and speed of the motors.
// The definitions of pins and directions are fixed,
// but you can change the motor's direction and speed.

// Input (int +255 to - 255), + indicate forward, - indicate reverse
// However I will limit the maximum input to 50 (Max Speed) it can be changed in verible named "Max_Speed"

class Motors_c {
  public:
    Motors_c() {
    }

//-------------------------------------------------
    // Define pin numbers
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

    // Define direction constants
#define FWD LOW
#define REV HIGH

    // Define maximum speed
#define Max_Speed 255


//------------------------------------------------


    void initialise() {
      pinMode(L_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(R_DIR, OUTPUT);

      digitalWrite(L_PWM, 0);
      digitalWrite(R_PWM, 0);
      digitalWrite(L_DIR, FWD);  // Initial pin state for motors
      digitalWrite(R_DIR, FWD);
    }


    void setMotorPower(float l_pwm, float r_pwm) {
      bool L_Direc = (l_pwm >= 0 ) ? FWD : REV;
      bool R_Direc = (r_pwm >= 0 ) ? FWD : REV;

      if (abs(l_pwm) <= Max_Speed) {
        digitalWrite(L_DIR, L_Direc);
        analogWrite(L_PWM, abs(l_pwm));
        //delay(5); // Left Motor
      } else {
        digitalWrite(L_DIR, L_Direc);
        analogWrite(L_PWM, Max_Speed);
        //delay(5); // If the inpur excees the max_speed, input max_speed
      }

      if (abs(r_pwm) <= Max_Speed) {
        digitalWrite(R_DIR, R_Direc);
        analogWrite(R_PWM, abs(r_pwm));
        //delay(5); // Right Motor
      } else {
        digitalWrite(R_DIR, R_Direc);
        analogWrite(R_PWM, Max_Speed);
        //delay(5); // If the inpur excees the max_speed, input max_speed
      }
    }
};



#endif