// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _SPEEDPID_H
#define _SPEEDPID_H

// Class to contain generic PID algorithm.
class SpeedPID_c {
  public:

    // Constructor, must exist.
    SpeedPID_c() {
    }

    //-------Varibles---------------------------------------------------------------------
    //--Speed PID varibles-----------------------------------------------------------------
    float PWM_output;

#define PID_update_ts 20  // PID update time in millis()
    unsigned long PID_ts; //PID time stamp

    float K_P;
    float K_I;
    float K_D;

    float P_term;
    float I_term;
    float D_term;

    float integral;
    float derivative;
    float error_last;

    //rotational velocity estimation varibles----------------------------------------------

#define rota_velo_update_ts 20   // time to updat the retaional velocity estimation (encoder revo velo calcu)
    float ave_el_spd;            // Rotation velocity "raw"
    long count_t_1;              // count stamp for right
    float lpf;                   // low pass filtered estimation of speed

    float alpha = 0.3;           //low-pass filter ratio
    unsigned long encoder_ts;    // rota velo counts timestamp

    //------------------------------------------------------------------------------------




    void initialise(float in_P, float in_I, float in_D) { //example: (count_e0, 80, 0.0001, 20);

      //initialise rotat-velo estiomation varibles
      ave_el_spd = 0.0;
      count_t_1 = 0;
      lpf = 0.0;
      encoder_ts = 0;

      //initialise Speed PID varibles
      PID_ts = millis();

      K_P = in_P;
      K_I = in_I;
      K_D = in_D;

      PWM_output = 0.0;

      P_term = 0.0;
      I_term = 0.0;
      D_term = 0.0;

      integral = 0.0;
      derivative = 0.0;
      error_last = 0.0;
    }




    float speedPID( long whichWheel, float demand ) {
      unsigned long current_ts = millis();
      unsigned long elapsed_t = current_ts - PID_ts;

      if (elapsed_t == 0) return PWM_output; //prevent d_t is zero

      if (elapsed_t > PID_update_ts) {
        PID_ts = current_ts; //update this literation of speedPID

        float measurement = rota_velo_est ( whichWheel );

        float error_sig = demand - measurement;
        //        Serial.print(", ");
        //        Serial.print("error_sig = ");
        //        Serial.println(error_sig);

        P_term = K_P * error_sig;

        integral = integral + error_sig * float(elapsed_t); //typecasting the d_t
        //        Serial.print(", ");
        //        Serial.print("dt = ");
        //        Serial.println(elapsed_t);


        I_term = K_I * integral;

        derivative = (error_sig - error_last) / float(elapsed_t); //typecasting the d_t
        D_term = K_D * derivative;
        error_last = error_sig; //update the error signal

        PWM_output = P_term + I_term - D_term;

      }

      return PWM_output;
    }




    //Input : count_e1 or count_e0 from encoder, asking which wheel you want to estimate?
    float rota_velo_est( long thisWheel) {
      // Record the time of this execution of the loop function
      unsigned long current_ts = millis();
      unsigned long elapsed_t = current_ts - encoder_ts;
      long count_t = thisWheel;

      if (elapsed_t == 0) return lpf; //prevent d_t is zero

      if (elapsed_t > rota_velo_update_ts) {

        // Calculate the change in encoder counts
        long count_diff = count_t - count_t_1;

        // Calculate the average speed as a floating-point value
        ave_el_spd = float(count_diff) / elapsed_t;

        // Low pass filter
        lpf = ( lpf * (1 - alpha ) ) + ( ave_el_spd * alpha ); // so we trust 1-alpha percent of our previous value and alpha percent of current reading

        // update the time and count stamps
        encoder_ts = current_ts;
        count_t_1 = thisWheel;
      }
      //Serial.print(lpf * 100);
      return lpf;
    }


};
#endif