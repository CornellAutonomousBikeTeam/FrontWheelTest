#ifndef Encoder_h
#define Encoder_h
#include <Arduino.h>

extern const int quad_A;
extern const int quad_B;
extern const int idx;
extern const unsigned int mask_quad_A;
extern const unsigned int mask_quad_B;
extern const unsigned int mask_idx;
extern const int REnot;
extern const int DE;

extern signed int x_offset;

extern signed int relativePos; //Read the relative position of the encoder
extern signed int indexValue; //Read the index value (Z channel) of the encoder
extern float fw_pos;
extern float fw_velocity;

float readFrontWheelState();

#endif //Encoder_h
