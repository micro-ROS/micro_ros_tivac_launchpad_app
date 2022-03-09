#ifndef ros_encoders_h
#define ros_encoders_h

#include "TivaQEI.h"

extern encoder_t encoder_left;
extern encoder_t encoder_right;

void zero_encoders(void);
void init_encoders(uint32_t period, bool left_swap, bool right_swap, bool left_ab, bool right_ab);
void measure_encoders();

#endif
