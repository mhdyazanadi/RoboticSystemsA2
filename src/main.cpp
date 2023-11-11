#include "kinematics.h"               //This main loop is the Turning PID!!!!
#include "motors.h"
#include "encoders.h"
#include "speedPID.h"
#include <EEPROM.h> //eeprom lib

Kinematics_c M_kinematics;
Motors_c M_motors;
SpeedPID_c M_speedPID_Left; //left wheel speed PID
SpeedPID_c M_speedPID_Right; // right wheel speed PID

//==Kinematics varibles ======================
float X_cor;
float Y_cor;
float Theta_cor;
//==========================================


//==SpeedPID varibles ========================
float leftSpeedInput; //output from speed PID (pwm)
float rightSpeedInput;

float l_demand;       //output from Heading PID
float r_demand;
//==========================================


//==TurnPID varibles==========================
float demand_theta;          //demand_theta, the theta you input in the turn PID as demand, need to be updated dynamcally (angle segmentation)
float current_theta;         //current_theta is the kinematic theta,

float turnBias;

#define PID_update_ts 20 // PID update time in millis()
unsigned long PID_ts; //PID time stamp

float K_P;
float K_I;
float K_D;

float P_term;
float I_term;
float D_term;
float totalOutput;

float integral;
float derivative;
float error_last;
//=========================================


//==Other varibles ========================
unsigned long taskDuration_ts;
#define taskDuration 2000

const int buzzerPin = 6;

unsigned long loop_ts;
unsigned long loop_ts_2;


float SET_ANGLE = 0;

//=============================EEPROM CODE=======================================

/*
 * This code segment manages EEPROM operations. 
 * An specific EEPROM address for each variable has been defined to  
 * prevent overlapping of addresses when being written to EEPROM.
 * This ensure no data corruption occurs during writing/reading.
 * The printRobotData() is used to output the values to the serial monitor
 * After the robot does the expriment, button A on the board should be pressed
 * to retrieve the data on the serial monitor. This allows the visualisation of
 * the robot's current state, including PID control parameters and kinematic data.
 * 
 */
#define BUTTON_A_PIN  14        //button A pin number
#define UPDATE_MS   1000        // Update period in milliseconds = 1sec
unsigned long eep_update_ts;    // Timestamp for the last EEPROM update

// EEPROM address constants, ensuring they dont ocverlapp
const int ADDR_CURRENT_THETA = 0;
const int ADDR_DEMAND_THETA = ADDR_CURRENT_THETA + sizeof(float);
const int ADDR_LEFT_SPEED_INPUT = ADDR_DEMAND_THETA + sizeof(float);
const int ADDR_RIGHT_SPEED_INPUT = ADDR_LEFT_SPEED_INPUT + sizeof(float);
const int ADDR_P_TERM = ADDR_RIGHT_SPEED_INPUT + sizeof(float);
const int ADDR_I_TERM = ADDR_P_TERM + sizeof(float);
const int ADDR_D_TERM = ADDR_I_TERM + sizeof(float);

void writeToEEPROM() {
  EEPROM.put(ADDR_CURRENT_THETA, current_theta);
  EEPROM.put(ADDR_DEMAND_THETA, demand_theta);
  EEPROM.put(ADDR_LEFT_SPEED_INPUT, leftSpeedInput);
  EEPROM.put(ADDR_RIGHT_SPEED_INPUT, rightSpeedInput);
  EEPROM.put(ADDR_P_TERM, P_term);
  EEPROM.put(ADDR_I_TERM, I_term);
  EEPROM.put(ADDR_D_TERM, D_term);
}


void readFromEEPROM() {
  EEPROM.get(ADDR_CURRENT_THETA, current_theta);
  EEPROM.get(ADDR_DEMAND_THETA, demand_theta);
  EEPROM.get(ADDR_LEFT_SPEED_INPUT, leftSpeedInput);
  EEPROM.get(ADDR_RIGHT_SPEED_INPUT, rightSpeedInput);
  EEPROM.get(ADDR_P_TERM, P_term);
  EEPROM.get(ADDR_I_TERM, I_term);
  EEPROM.get(ADDR_D_TERM, D_term);
}

void printRobotData() {
  Serial.print("Current Theta: "); Serial.println(current_theta);
  Serial.print("Demand Theta: "); Serial.println(demand_theta);
  Serial.print("Left Speed Input: "); Serial.println(leftSpeedInput);
  Serial.print("Right Speed Input: "); Serial.println(rightSpeedInput);
  Serial.print("P Term: "); Serial.println(P_term);
  Serial.print("I Term: "); Serial.println(I_term);
  Serial.print("D Term: "); Serial.println(D_term);
  Serial.println("----");
}
//=============================EEPROM CODE=======================================


void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println(F("***RESET***"));

  // Initialize EEPROM address and timestamp
  //eeprom_address = 0;
  eep_update_ts = millis();  // Initialize the EEPROM update timestamp
  
  pinMode(BUTTON_A_PIN, INPUT_PULLUP); 
  pinMode(buzzerPin, OUTPUT);

  //Encoder IRS set up
  setupEncoder0();
  setupEncoder1();

  //Initialising classes file
  M_kinematics.initialise();
  M_motors.initialise();
  mainLevelInitialise();

  //Initialising PID parameters (Changing P I D gains here)
  M_speedPID_Left.initialise ( 100.0, 0.7, 100.0); // P = 50.0, I = 0.5, D = 100.0 works well in Assessment 1, but need to be re tuned for small angle rotation
  M_speedPID_Right.initialise( 100.0, 0.7, 100.0);

  turnToAnglePID_initialise (0.08, 0.0003, 0.004); // turnPID P = 0.08, I = 0.0003, D = 0.004


  scoreBeep();
  delay(3000);
  scoreBeep();
  scoreBeep();  //Start operation

  //testing================
 // l_demand = 0.2;   //0.2 seems to be a good value 
    demand_theta = 3.14 / 36;
  //========================



  //==================================================================================================================================
  //==================================================================================================================================
  //==ANGLE REQUESTED TO TURN MODIFIY HERE ===========================================================================================

  //SET_ANGLE = 0.0;

  //==================================================================================================================================
  //==================================================================================================================================
  //==================================================================================================================================

  //Time initialise ======================================
  //taskDuration_ts = millis(); //record starting time
  M_speedPID_Left.PID_ts = millis();
  M_speedPID_Right.PID_ts = millis();
  PID_ts = millis();

  
  loop_ts = millis();
  loop_ts_2 = millis();

  // Read data from EEPROM to initialise variables from EERPOM
  readFromEEPROM();
  printRobotData();
  
}







void loop() {
  // Check if Button A is pressed
  if (digitalRead(BUTTON_A_PIN) == LOW) { 
    // Button A pressed, read data from EEPROM and print
    readFromEEPROM();
    printRobotData();

    // Add a small delay to debounce the button press and prevent multiple reads
    delay(200); 
  }

  M_kinematics.updateKinematics();                    // keep Kinematics updating, the updating rate, pls see the class file

  X_cor = M_kinematics.X_t_I;
  Y_cor = M_kinematics.Y_t_I;
  Theta_cor = M_kinematics.theta_t_I;    //update varible value to the main level.
  current_theta =  Theta_cor;            // update kinematic theta for turnPID as current_theta

  turnBias = turnToAnglePID (demand_theta, current_theta);
    l_demand = 0 + turnBias;
    r_demand = 0 - turnBias;
  //
  leftSpeedInput =  M_speedPID_Left.speedPID(count_e1, l_demand);  //update speedPID

  rightSpeedInput = M_speedPID_Right.speedPID(count_e0, r_demand);

  /** 
   * After updating
   * control and kinematics variables, 
   * write to EEPROM
   */
  if (millis() - eep_update_ts > UPDATE_MS) {
    eep_update_ts = millis(); // Update the timestamp

    // Call the function to write data to EEPROM
    writeToEEPROM();
  }
  
  
  M_motors.setMotorPower (leftSpeedInput, rightSpeedInput); // PWMinput to the motor
  // do dobule check the MaxSpeed allowed for the motor in the class file (MaxPWM input)
  // default is 50


  //  if ( (millis() - taskDuration_ts) > taskDuration ) {
  //    while (1) {
  //      //put data retriving function here
  //    }
  //  }

  //while ()

  //testing =============================================
//    unsigned long current_ts_2 = millis();
//    unsigned long elapsed_t_2 = current_ts_2 - loop_ts_2;
//    if (elapsed_t_2 > 2000) {
//      loop_ts_2 = current_ts_2;
//      l_demand = l_demand * -1.0;
//    }
  //  //this is for testing (above) it flips l_demand sign every 5 sec, so l_demand is +1.2 5sec later, it is -1.2
  //======================================================

  //testing--------------------------
  
    


  //  Serial.print(F("l_demand "));

  //  Serial.print(F(",ldem "));
  //  Serial.print(l_demand);
  //  Serial.print(F(",rdem"));
  //  Serial.print(r_demand);



  //--Controller output printing------
  //  Serial.print(F("SETANGLE "));
  //  Serial.print(SET_ANGLE);
  //  Serial.print(F(",ldem "));
  //  Serial.print(l_demand);
  //  Serial.print(F(",rdem"));
  //  Serial.print(r_demand);

  //--kinematics result printing-----
  //  Serial.print(F(",X "));
  //  Serial.print(X_cor);
  //  Serial.print(F(",Y "));
  //  Serial.print(Y_cor);
  //  Serial.print(F(",T "));
  //  Serial.print(Theta_cor);

  //--Controller output printing------
  //  Serial.print(F(",tBs "));
  //  Serial.print(turnBias);
  //  Serial.print(F(",LP "));
  //  Serial.print(leftSpeedInput);
  //  Serial.print(F(",RP"));
  //  Serial.print(rightSpeedInput);

  //--turnPID internal parameter printing----
  //  Serial.print(F(",tP "));
  //  Serial.print(P_term);
  //  Serial.print(F(",tI "));
  //  Serial.print(I_term);
  //  Serial.print(F(",tD"));
  //  Serial.print(D_term);

  //--Left SpeedPID internal parameter printing----
  //      Serial.print(F(",LSP "));
  //      Serial.print( M_speedPID_Left.P_term);
  //  Serial.print(F(",LSI "));
  //  Serial.print( M_speedPID_Left.I_term);
  //      Serial.print(F(",LSD"));
  //      Serial.print( M_speedPID_Left.D_term);
  //    Serial.print(F(",Lvelo"));
  //  Serial.print(",");
  //  Serial.println( M_speedPID_Left.integral);

  
   
  //  Serial.print(F(",encoC"));
  //  Serial.print( M_speedPID_Left.count_t_1);


  //--Right SpeedPID internal parameter printing----
  //  Serial.print(F(",RSP "));
  //  Serial.print( M_speedPID_Right.P_term);
  //  Serial.print(F(",RSI "));
  //  Serial.print( M_speedPID_Right.I_term);
  //  Serial.print(F(",RSD"));
  //  Serial.print( M_speedPID_Right.D_term);
  //  Serial.print(F(",Rvelo"));
  //  Serial.print( M_speedPID_Right.lpf);


  //Serial.print(F(","));
  //Serial.print();
  //Serial.print(F(","));
  //Serial.print();


  //Serial.print(F("\n")); // Changed from /n to \n to print a newline.


}




float turnToAnglePID (float demand_turn, float measurement_turn) {
  unsigned long current_ts = millis();
  unsigned long elapsed_t = current_ts - PID_ts;

  if (elapsed_t == 0) return totalOutput; //prevent d_t is zero

  if (elapsed_t > PID_update_ts) {
    PID_ts = current_ts;  //update PID timestamp

    float error_sig = demand_turn - measurement_turn;

    P_term = K_P * error_sig;

    integral = integral + error_sig * float(elapsed_t); //typecasting the d_t
    I_term = K_I * integral;

    derivative = (error_sig - error_last) / float(elapsed_t); //typecasting the d_t
    D_term = K_D * derivative;
    error_last = error_sig; //update the error signal

    totalOutput = P_term + I_term - D_term;
  }
  return totalOutput;
}







void turnToAnglePID_initialise(float in_P, float in_I, float in_D) {
  PID_ts = millis();

  K_P = in_P;
  K_I = in_I;
  K_D = in_D;

  P_term = 0.0;
  I_term = 0.0;
  D_term = 0.0;
  totalOutput = 0.0;

  integral = 0.0;
  derivative = 0.0;
  error_last = 0.0;
}






void mainLevelInitialise() {

  //Kinematics varibles ======================
  X_cor = 0.0;
  Y_cor = 0.0;
  Theta_cor = 0.0;
  //==========================================

  //SpeedPID varibles ========================
  leftSpeedInput = 0.0; //output from speed PID
  rightSpeedInput = 0.0;
  l_demand = 0.0; //output from Heading PID
  r_demand = 0.0;
  //==========================================

  //TurnPID varibles==========================
  demand_theta = 0.0;
  current_theta = 0.0;         //current_theta is the kinematic theta,
  turnBias = 0.0;
  PID_ts = 0; //PID time stamp
  //===========================================


  //Other varibles ============================
  taskDuration_ts = 0;
  loop_ts = 0;
  loop_ts_2 = 0;
  //===========================================
}






void scoreBeep() {
  // First spike
  tone(buzzerPin, 1500, 50);  // tone(pin, frequency, duration)
  delay(100);

  // Second spike
  tone(buzzerPin, 2000, 50);
  delay(100);
}