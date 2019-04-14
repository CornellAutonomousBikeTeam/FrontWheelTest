#include "FrontWheel.h"
#include "Encoder.h"

#define LED_RED 22
#define LED_YLW 35
#define LED_BLU 36


void setup() {
  Serial.begin(115200);

  pinMode(REnot, OUTPUT);
  pinMode(DE, OUTPUT);

  // activate peripheral functions for quad pins
  REG_PIOB_PDR = mask_quad_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_A;   // choose peripheral option B
  REG_PIOB_PDR = mask_quad_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_B;   // choose peripheral option B
  REG_PIOB_PDR = mask_idx;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_idx;   // choose peripheral option B

  REG_PMC_PCER0 = (1 << 27) | (1 << 28) | (1 << 29); // activate clock for TC0 and TC1
  REG_TC0_CMR0 = 5; // select XC0 as clock source and set capture mode
  REG_TC0_BMR = (1 << 9) | (1 << 8) | (1 << 12); // activate quadrature encoder and position measure mode, no filters
  REG_TC0_QIER = 1; // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)

  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;

  //setup Motor Outputs
  pinMode(FRONT_DIR_PIN, OUTPUT);
  pinMode (FRONT_PWM_PIN, OUTPUT);

  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YLW, OUTPUT);
  pinMode(LED_BLU, OUTPUT);

  calibrate_front_wheel();
  delay(500);
  readPidCoeffs();
  doTestProgram();
  /*
  while(true){
    readPidCoeffs();
    //keepAtZero(100000);
  }
  */
}

void readPidCoeffs() {
  Serial.println("Enter Kp and Kd");
  digitalWrite(LED_BLU, HIGH);
  Kp = 0;
  do {
    while(!Serial.available()) {}
    Kp = Serial.parseFloat();
    Kd = Serial.parseFloat();
    Serial.print(Kp);
    Serial.print(' ' );
    Serial.println(Kd);
    digitalWrite(LED_BLU, LOW);
  } while(Kp == 0);
}
/*
void keepAtZero(long iters) {
  for(long i = 0; i < iters; i++) {
    readFrontWheelState();
    fw_pos_controller(0);
  }
  set_motor_velocity(0);
}*/

void doTestProgram() {
  digitalWrite(LED_YLW, HIGH);
  unsigned long start_time = millis();
  unsigned long done_zeroing_time = start_time + 2000;
  long curr_time;
  long prev_time;
  while(millis() < done_zeroing_time) {
    curr_time = millis();
    readFrontWheelState();
    fw_pos_controller(0, curr_time-prev_time);
    prev_time = curr_time;
  } 
  set_motor_velocity(0);
  delay(100);
  int counter = 0;
  int arr_idx = 0;
  float desired_pos = M_PI/4;
  const int N = 400;
  long times[N];
  float positions[N];
  int step_idx = N/4;
  while(arr_idx<N) {
    curr_time = millis();
    readFrontWheelState();
    if(arr_idx>=step_idx)fw_pos_controller(desired_pos, curr_time - prev_time);
    counter = (counter+1) & 0xFF;
    if(counter==0) {
      times[arr_idx] = millis();
      positions[arr_idx] = fw_pos;
      arr_idx++;
    }
    prev_time = curr_time;
  }
  set_motor_velocity(0);
  digitalWrite(LED_YLW, HIGH);
  Serial.println("0\t0\t" + String(positions[0]));
  for(int i=1; i < N; i++) {
    Serial.print(String(times[i]-times[0]) + "\t");
    Serial.print((i>=step_idx)*(-M_PI/4),4);
    Serial.print("\t");
    Serial.println(positions[i],4);
  }
  set_motor_velocity(0);
  digitalWrite(LED_YLW, LOW);
}

void loop() {}
