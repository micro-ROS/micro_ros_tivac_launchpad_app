#include "TivaQEI.h"

int32_t encdir_left;
int32_t encdir_right;

uint32_t vel_left;
uint32_t vel_right;

uint32_t pos_left;
uint32_t pos_right;

int16_t delta_left;
int16_t delta_right;

encoder_t encoder_left = {};
encoder_t encoder_right = {};

uint32_t pos;

void zero_encoders(void) {
  set_encoder_position(&encoder_left, 0L);
  set_encoder_position(&encoder_right,0L);
}

void init_encoders(uint32_t period, bool left_swap, bool right_swap, bool left_ab, bool right_ab) {

  // period is ui32SysClkFreq / update_frequency
  init_encoder(&encoder_left, QEI0_BASE, period, left_swap, left_ab);
  init_encoder(&encoder_right, QEI1_BASE, period, right_swap, right_ab);

  zero_encoders();

}

void measure_encoders() {

  // encoder direction
  encdir_left = get_direction(&encoder_left);
  encdir_right = get_direction(&encoder_right);

  // encoder velocity
  vel_left = get_velocity(&encoder_left);
  vel_right = get_velocity(&encoder_right);

  // encoder position
  pos_left = get_position(&encoder_left);
  pos_right = get_position(&encoder_right);

  // encoder delta
  delta_left = get_delta(&encoder_left);
  delta_right = get_delta(&encoder_right);

}