#ifndef ros_encoders_h
#define ros_encoders_h

#include "TivaQEI.h"

#ifdef __cplusplus
extern "C"
{
#endif

int32_t encdir_left;
int32_t encdir_right;

uint32_t vel_left;
uint32_t vel_right;

uint32_t pos_left;
uint32_t pos_right;

int16_t delta_left;
int16_t delta_right;

Encoder encoder_left;
Encoder encoder_right;

uint32_t pos;

void zero_encoders(void) {
  encoder_left.set_encoder_position(0L);
  encoder_right.set_encoder_position(0L);
}

void init_encoders(uint32_t period, bool left_swap, bool right_swap, bool left_ab, bool right_ab) {

  // period is ui32SysClkFreq / update_frequency
  encoder_left.init_encoder(QEI0_BASE, period, left_swap, left_ab);
  encoder_right.init_encoder(QEI1_BASE, period, right_swap, right_ab);

  zero_encoders();

}

void measure_encoders() {

  // encoder direction
  encdir_left = encoder_left.get_direction();
  encdir_right = encoder_right.get_direction();

  // encoder velocity
  vel_left = encoder_left.get_velocity();
  vel_right = encoder_right.get_velocity();

  // encoder position
  pos_left = encoder_left.get_position();
  pos_right = encoder_right.get_position();

  // encoder delta
  delta_left = encoder_left.get_delta();
  delta_right = encoder_right.get_delta();

}

#ifdef __cplusplus
}
#endif

#endif
