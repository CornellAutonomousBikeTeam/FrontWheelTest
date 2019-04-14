#include "Encoder.h"

/*Variables*/
const int quad_A = 2; // front wheel encoder, channel A
const int quad_B = 13; // front wheel encoder, channel B
const int idx = 60; // front wheel encoder, channel Z
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);
const unsigned int mask_idx = digitalPinToBitMask(idx);
const int REnot = 3;
const int DE = 4;

signed int x_offset = 0;

signed int relativePos = REG_TC0_CV0;
signed int indexValue = REG_TC0_CV1;
float fw_pos = 0;
float fw_velocity = 0;

unsigned long prev_time = 0;
float prev_pos = 0;

/*
 * Sets five global variables: relativePos, indexValue, fw_pos, fw_velocity, prev_time, and prev_pos.
 * prev_time and prev_pos are only used in this function, and store the most recent time this function was called and the front wheel position at that time, respectively.
 */
float readFrontWheelState() {
  relativePos = REG_TC0_CV0;
  indexValue = REG_TC0_CV1;
  fw_pos = (((relativePos - x_offset) * 0.02197 * M_PI) / 180); //Angle (rad)
  unsigned long curr_time = micros();

  // Factor of 1,000,000 in the numerator because we want a value in rad/sec, not rad/microsec
  fw_velocity = 1000000 * (fw_pos - prev_pos) / (curr_time - prev_time);
  prev_pos = fw_pos;
  prev_time = curr_time;
}
