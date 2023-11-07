// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
// kinematics.h
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <Arduino.h>


class Kinematics_c {
  private:
    float x_position = 0;
    float y_position = 0;
    float theta = 0; // Robot's orientation
    const float len = 96.0; // length between wheels as in documentation
    const float radius = 16.0; // radius of wheel as in documentation
    float ticks_per_revolution = 12.0; //as in documentation
    unsigned long previous_time = 0;
    float previous_count_Left = 0;
    float previous_count_Right = 0;

  public:
    // Constructor
    Kinematics_c() { }

    // Function to update your kinematics
    void update(float count_Left, float count_Right, float delta_t) {
      float delta_count_Left = count_Left - previous_count_Left;
      float delta_count_Right = count_Right - previous_count_Right;

      float V_left = 2 * PI * radius * delta_count_Left / (ticks_per_revolution * delta_t);
      float V_right = 2 * PI * radius * delta_count_Right / (ticks_per_revolution * delta_t);

      // Calculate linear and angular velocities
      float V = (V_left + V_right) / 2.0;
      float omega = (V_right - V_left) / len;

      // Compute changes in pose
      float delta_x = V * cos(theta) * delta_t;
      float delta_y = V * sin(theta) * delta_t;
      float delta_theta = omega * delta_t;

      // Update pose
      x_position += delta_x;
      y_position += delta_y;
      theta += delta_theta;

      // Normalize theta between -PI and PI
      if (theta > PI) {
        theta -= 2 * PI;
      } else if (theta < -PI) {
        theta += 2 * PI;
      }

      //update the previous counts
      previous_count_Left = count_Left;
      previous_count_Right = count_Right;
    }

    float getX() { return x_position; }
    float getY() { return y_position; }
    float getTheta() { return theta; }
};

#endif



 // Print values for validation
 // Serial.print("X Position: ");
 // Serial.println(x_position);
 // Serial.print("Y Position: ");
 // Serial.println(y_position);
 // Serial.print("Theta (radians): ");
 // Serial.println(theta);
 // Serial.print("V_left (mm/s): ");
 // Serial.println(V_left);
 // Serial.print("V_right (mm/s): ");
 // Serial.println(V_right);
 // Serial.print("V (mm/s): ");
 // Serial.println(V);
 //Serial.print("Omega (rad/s): ");
 // Serial.println(omega);
 // Serial.println("-------------------");
