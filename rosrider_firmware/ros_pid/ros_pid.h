#ifndef ros_pid_h
#define ros_pid_h

#include "PID.h"

float target_rpm_left, target_rpm_right;
float output_left, output_right;
float current_rpm_left, current_rpm_right;

PID pid_left(&current_rpm_left, &target_rpm_left, &output_left, left_Kp, left_Ki, left_Kd);
PID pid_right(&current_rpm_right, &target_rpm_right, &output_right, right_Kp, right_Ki, right_Kd);

void update_pid(double dt) {

  measure_encoders();

  // notice to get signed velocity, we must multiply vel by encdir
  int32_t c_left = vel_left * encdir_left;
  int32_t c_right = vel_right * encdir_right;

  current_rpm_left = c_left * ROUNDS_PER_MINUTE;
  current_rpm_right = c_right * ROUNDS_PER_MINUTE;

  // update pid
  pid_left.update(dt);
  pid_right.update(dt);

  // get pwm values
  pwm_left = (int16_t) output_left;
  pwm_right = (int16_t) output_right;

  // set direction
  if(pwm_left >= 0) { left_phase(FORWARD); } else if(pwm_left < 0) { left_phase(REVERSE); }
  if(pwm_right >= 0) { right_phase(FORWARD); } else if(pwm_right < 0) { right_phase(REVERSE); }

  // set pwm
  left_pwm(abs(pwm_left));
  right_pwm(abs(pwm_right));

}

void update_pid_parameters() {
  pid_left.set_tunings(left_Kp, left_Ki, left_Kd);
  pid_right.set_tunings(right_Kp, right_Ki, right_Kd);
}

#endif