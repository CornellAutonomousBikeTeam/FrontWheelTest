#include "FrontWheel.h"
#include <math.h>

const long interval = 10000; //timed loop interval, in microseconds

float desired_steer = 0;
float desired_lean = 0;

int Kp = 500;
int Ki = 0;
int Kd = 00;

int maxfront_PWM = 110;

const int k1 = 70; //phi = lean
const int k2 = 10; //was previously 21 //phidot=lean rate
const int k3 = -20; //delta=steer

float last_error = 0;

/*
 * Set a motor velocity. velocity ranges from -100 to 100. Positive means
 * clockwise; negative means counterclockwise.
 */
void set_motor_velocity(int velocity) {
  if (velocity > 0) {
    digitalWrite(FRONT_DIR_PIN, LOW); 
  } else {
    digitalWrite(FRONT_DIR_PIN, HIGH);
  }

  int abs_velocity = abs(velocity);

  // Limit to 100, or roughly 40% duty cycle
  if(abs_velocity > 100) {
    abs_velocity = 100;
  }

  analogWrite(FRONT_PWM_PIN, abs_velocity);  
}

/*
 * Calculates x_offset by rotating the front motor until the index pin
 * (aka channel Z on the encoder) goes to 0.
 */
void calibrate_front_wheel() {
  signed int y = REG_TC0_CV1;
  int oldIndex = y;
  set_motor_velocity(-40);

  //Front wheel calibration loop
  while(y==oldIndex) {
    y = REG_TC0_CV1;
  }

  x_offset = REG_TC0_CV0;   //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  set_motor_velocity(0);
}

/*
 * Runs a PD controller to rotate the front wheel to desired_pos
 * (specified in radians). Convention: desired_pos is 0 when the wheel
 * points straight forwards, and increases clockwise.
 */
void fw_pos_controller(float desired_pos, long time_diff) {

  //P term
  //calculate position error (radians)
  float pos_error = desired_pos + fw_pos;

  //scaled positional error
  //position scaling factor K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), 
  //and dividing by theoretical max absolute value of angle (Pi/2).
  //This means with angles in that range, 100 will be the max PWM value outputted to the motor
  float sp_error =  (Kp*pos_error);

  //D term
  //calculate velocity error

  //Calculated as target_velocity - current_velocity where target velocity is always 0
  //scaled velocity error
  //float sv_error =  (-Kd*fw_velocity);
  float sv_error = -Kd*((pos_error - last_error)/time_diff);
  last_error = pos_error;

  float total_error =  sp_error + sv_error ;
  //Serial.println(String(desired_pos)+'\t'+String(time_diff)+'\t'+String(fw_pos)+'\t'+String(sp_error)+'\t'+String(sv_error)+'\t'+String(total_error));
  set_motor_velocity((int)total_error);
}
