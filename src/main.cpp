
#include "kinematics.h"               //This main loop is the Turning PID!!!!
#include "motors.h"
#include "encoders.h"
#include "speedPID.h"
#include <EEPROM.h> //EEPROM LIBRARY

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
float leftSpeedInput; //output from speed PID
float rightSpeedInput;

float l_demand;       //output from Heading PID
float r_demand;
//==========================================


//==TurnPID varibles==========================
float demand_theta;       //demand_theta, the theta you input in the turn PID as demand, need to be updated dynamcally (angle segmentation)
float current_theta;      //current_theta is the kinematic theta,

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
//=========================================

//=============================================EEPROM IMPLEMENTATION ============================================


#define BUTTON_A_PIN  14  // Button A for Mode Selection
#define BUTTON_B_PIN  30  // Button B for Reporting

// Modes
#define MODE_RUNNING_EXPERIMENT  0
#define MODE_REPORTING_RESULTS   1

int currentMode = MODE_RUNNING_EXPERIMENT;  // Default mode

# define UPDATE_MS   1000
unsigned long eep_update_ts;
int eeprom_address;


struct RobotData {
  float current_theta;
  float p_term;
  float i_term;
  float d_term;
  int pwm_left;
  int pwm_right;
} robotData;


void writeToEEPROM() {
  int eeAddress = 0; // Start address to write
  EEPROM.put(eeAddress, robotData);
}

void readFromEEPROM() {
  int eeAddress = 0; // Start address to read
  EEPROM.get(eeAddress, robotData);
}

void printRobotData() {
  Serial.print("Current Theta: "); Serial.println(robotData.current_theta);
  Serial.print("P Term: "); Serial.println(robotData.p_term);
  Serial.print("I Term: "); Serial.println(robotData.i_term);
  Serial.print("D Term: "); Serial.println(robotData.d_term);
  Serial.print("PWM Left: "); Serial.println(robotData.pwm_left);
  Serial.print("PWM Right: "); Serial.println(robotData.pwm_right);
  Serial.println("----");
}

//=============================================EEPROM IMPLEMENTATION ============================================



void setup() {

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("***RESET***"));


  //=============================================

  // Mode selection
  Serial.println("A = Run Experiment, B = Report Results");
  while (digitalRead(BUTTON_A_PIN) == HIGH && digitalRead(BUTTON_B_PIN) == HIGH) {
    // Waiting for button press
  }
  if (digitalRead(BUTTON_A_PIN) == LOW) {
    currentMode = MODE_RUNNING_EXPERIMENT;
    Serial.println("Mode: Running Experiment");
  } else if (digitalRead(BUTTON_B_PIN) == LOW) {
    currentMode = MODE_REPORTING_RESULTS;
    Serial.println("Mode: Reporting Results");
  }


  //===============================================

  pinMode(buzzerPin, OUTPUT);

  //Encoder IRS set up
  setupEncoder0();
  setupEncoder1();

  //Initialising classes file
  M_kinematics.initialise();
  M_motors.initialise();
  mainLevelInitialise();

  //Initialising PID parameters (Changing P I D gains here)
  M_speedPID_Left.initialise (count_e1, 50.0, 0.5, 100.0); // P = 50.0, I = 0.5, D = 100.0 works well in Assessment 1, but need to be re tuned for small angle rotation
  M_speedPID_Right.initialise(count_e0, 50.0, 0.5, 100.0);

  turnToAnglePID_initialise (0.08, 0.0003, 0.004); // turnPID P = 0.08, I = 0.0003, D = 0.004


  scoreBeep();
  delay(3000);
  scoreBeep();
  scoreBeep();  //Start operation

  //testing================
  //  loop_ts = millis();
  //  loop_ts_2 = millis();
  //  l_demand = 1.2;
  //========================


  //==================================================================================================================================
  //==================================================================================================================================
  //==ANGLE REQUESTED TO TURN MODIFIY HERE ===========================================================================================

  SET_ANGLE = 0.0;

  //==================================================================================================================================
  //==================================================================================================================================
  //==================================================================================================================================



  taskDuration_ts = millis(); //record starting time
}





void loop() {

  if (currentMode == MODE_RUNNING_EXPERIMENT) {

    M_kinematics.updateKinematics();    // keep Kinematics updating, the updating rate, pls see the class file

    X_cor = M_kinematics.X_t_I;
    Y_cor = M_kinematics.Y_t_I;
    Theta_cor = M_kinematics.theta_t_I;    //update varible value to the main level.
    current_theta =  Theta_cor;            // update kinematic theta for turnPID as current_theta

    turnBias = turnToAnglePID (demand_theta, current_theta);
    l_demand = 0 + turnBias;
    r_demand = 0 - turnBias;

    leftSpeedInput =  M_speedPID_Left.speedPID(l_demand);  //update speedPID
    rightSpeedInput = M_speedPID_Right.speedPID(r_demand);

    M_motors.setMotorPower (leftSpeedInput, rightSpeedInput); // PWMinput to the motor

    //===================EEPROM UPDATING MEMORY====================================================
    robotData.current_theta = current_theta;
    robotData.p_term = P_term;
    robotData.i_term = I_term;
    robotData.d_term = D_term;
    robotData.pwm_left = leftSpeedInput;
    robotData.pwm_right = rightSpeedInput;

    if (millis() - eep_update_ts > UPDATE_MS) {
      eep_update_ts = millis();

      // Write to EEPROM
      writeToEEPROM();

      // Increment eeprom_address with caution due to EEPROM size and write limits
      eeprom_address += sizeof(RobotData);
      if (eeprom_address > EEPROM.length() - sizeof(RobotData)) {
        eeprom_address = 0; // Reset address or handle as needed
      }
    }


    printRobotData();

    //========================================================================================


  } else if (currentMode == MODE_REPORTING_RESULTS) {
    for (int address = 0; address <= EEPROM.length() - sizeof(RobotData);
         address += sizeof(RobotData)) {
      EEPROM.get(address, robotData);
      printRobotData();
      delay(1000); // Delay for readability
    }
  }

}
// do dobule check the MaxSpeed allowed for the motor in the class file (MaxPWM input)
// default is 50


//  if ( (millis() - taskDuration_ts) > taskDuration ) {
//    while (1) {
//      //put data retriving function here
//    }
//  }


// Testing the speedPID flip the demands sign every 2 sec

//  unsigned long current_ts_2 = millis();
//  unsigned long elapsed_t_2 = current_ts_2 - loop_ts_2;
//  if (elapsed_t_2 > 5000) {
//    loop_ts_2 = current_ts_2;
//    l_demand = l_demand * -1.0;
//  }


//--Controller output printing------
//  Serial.print(F("SETANGLE "));
//  Serial.print(SET_ANGLE);
//Serial.print(F(",ldem "));
//Serial.print(l_demand);
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
//  Serial.print(F(",LSP "));
//  Serial.print( M_speedPID_Left.P_term);
//  Serial.print(F(",LSI "));
//  Serial.print( M_speedPID_Left.I_term);
//  Serial.print(F(",LSD"));
//  Serial.print( M_speedPID_Left.D_term);
//Serial.print(F(",Lvelo"));
//Serial.print( M_speedPID_Left.lpf);


//--Right SpeedPID internal parameter printing----
//  Serial.print(F(",RSP "));
//  Serial.print( M_speedPID_Right.P_term);
//  Serial.print(F(",RSI "));
//  Serial.print( M_speedPID_Right.I_term);
//  Serial.print(F(",RSD"));
//  Serial.print( M_speedPID_Right.D_term);
//  Serial.print(F(",Rvelo"));
//  Serial.print( M_speedPID_Left.lpf);


//Serial.print(F(","));
//Serial.print();
//Serial.print(F(","));
//Serial.print();




//Serial.print(F("\n"));
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



//====================== EEPROM METHOD ===================

//====================== EEPROM METHOD ===================










