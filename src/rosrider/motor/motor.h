#ifndef ros_motors_h
#define ros_motors_h

#include <stdint.h>
#include <stdbool.h>

#include "motor_enums.h"

#define PWM_FREQUENCY 400

void left_rev(bool reverse);
void right_rev(bool reverse);
void left_phase(motor_dir_t dir);
void right_phase(motor_dir_t dir);
void left_mode2(bool mode2);
void right_mode2(bool mode2);
void left_mode1(bool mode1);
void right_mode1(bool mode1);
void left_pwm(uint16_t pwm);
void right_pwm(uint16_t pwm);
void init_motors(bool _left_rev, bool _right_rev);

#endif