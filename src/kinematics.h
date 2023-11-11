// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "motors.h"
#include "encoders.h"

// Class to track robot position.
class Kinematics_c {
  public:

    // Constructor, must exist.
    Kinematics_c() {

    }


    //---------------------------------------------------------------------
#define Pi 3.1415926 //Pi = 3.3 using PID, = 3.10 using directe motor input
#define ENCODER_UPDATE 10 // time interval to calculate kinemitcs, in millis
#define l_W2center 142 // / 285/2, the distance from the wheels to the center in encoder counts, manually measured

    long left_wheel = 0;
    long right_wheel = 0;
    unsigned long encoder_ts;
    float X_dot_R;
    float theta_dot_R;
    float X_t_I;
    float Y_t_I;
    float theta_t_I;

    //-------------------------------------------------------------------




    // Use this function to update
    // your kinematics
    void updateKinematics() {

      unsigned long current_ts;
      current_ts = millis();

      //Update the encoder count every ENCODER_UPDATE  millis
      unsigned long elapsed_t = current_ts - encoder_ts;
      if (elapsed_t > ENCODER_UPDATE) {
        long left_wheel_t_1 = left_wheel;
        long right_wheel_t_1 = right_wheel;
        // update last reading

        left_wheel = count_e1;
        right_wheel = count_e0;
        // update current reading

        // Record when this execution happened.
        // for future iterations of loop()
        encoder_ts = millis();


        float phi_l = (left_wheel - left_wheel_t_1) ;
        float phi_r = (right_wheel - right_wheel_t_1) ;

        X_dot_R = phi_l / 2 + phi_r / 2;

        theta_dot_R = (phi_l / 2 - phi_r / 2) / l_W2center;

        X_t_I = X_t_I + (X_dot_R * cos (theta_t_I) );
        Y_t_I = Y_t_I - (X_dot_R * sin (theta_t_I) );
        theta_t_I = theta_t_I + theta_dot_R;

        //        Normalize to [0, 2 * Pi] as you're currently doing
        //        while (theta_t_I >= 2 * Pi) {
        //        theta_t_I -= 2 * Pi;
        //      }
        //        while (theta_t_I < 0) {
        //        theta_t_I += 2 * Pi;
        //      }



//        Serial.print("X = ");
//        Serial.print(X_t_I, 6);
//        Serial.print(", Y = ");
//        Serial.println(Y_t_I, 6);
        // Serial.print(", theta_t_I = ");
        // Serial.println(theta_t_I, 6);
        // Serial.print(", x = ");
        // Serial.print(X_dot_R);
        // Serial.print(", theta_dot_R = ");
        // Serial.println(theta_dot_R);
        // Serial.print("left:");
        // Serial.print(left_wheel);
        // Serial.print(" , right:");
        // Serial.println(right_wheel);

      }
    }



    void initialise() {
      encoder_ts = millis(); // initialise start_time of encoder

      left_wheel = 0;
      right_wheel = 0; //count initilise for wheels

      X_dot_R = 0.0;
      theta_dot_R = 0.0; //initialise the local cord

      X_t_I = 0;
      Y_t_I = 0;
      theta_t_I = 0; //initialise the Globle cord

    }
};



#endif