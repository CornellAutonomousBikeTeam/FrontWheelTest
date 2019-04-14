#ifndef FrontWheel_h
#define FrontWheel_h

#include <SPI.h>
#include <math.h>
#include "Encoder.h"

//PID controller on the front wheel
extern int Kp;
extern int Ki;
extern int Kd;

extern const long interval;

//Front Motor
// originally just PWM_front
#define FRONT_PWM_PIN 9
// originally just PIN
#define FRONT_DIR_PIN 46

extern float desired_steer;
extern float desired_lean;

extern int maxfront_PWM; //Define max front wheel PWM

//Balance Control constants
extern const int k1; //phi = lean
extern const int k2; //was previously 21 //phidot=lean rate
extern const int k3; //delta=steer

void set_motor_velocity(int);

void calibrate_front_wheel();

void fw_pos_controller(float, long);

#endif //PID_h
