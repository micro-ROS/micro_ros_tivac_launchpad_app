
#ifndef MICROROS_PARAMETERS_H_
#define MICROROS_PARAMETERS_H_

#include <rclc_parameter/rclc_parameter.h>

#define PARAMETERS \
  X(update_rate_ms, RCLC_PARAMETER_INT) \
  X(motor_polarity, RCLC_PARAMETER_BOOL) \
  X(enc_direction, RCLC_PARAMETER_BOOL) \
  X(enc_phase, RCLC_PARAMETER_BOOL) \
  X(enc_ppr, RCLC_PARAMETER_INT) \
  X(wheel_dia, RCLC_PARAMETER_DOUBLE) \
  X(base_width, RCLC_PARAMETER_DOUBLE) \
  X(motor_deadzone, RCLC_PARAMETER_INT) \
  X(main_current_limit, RCLC_PARAMETER_DOUBLE) \
  X(motor_current_limit, RCLC_PARAMETER_DOUBLE) \
  X(battery_volts_low, RCLC_PARAMETER_DOUBLE) \
  X(battery_volts_high, RCLC_PARAMETER_DOUBLE) \
  X(max_rpm, RCLC_PARAMETER_INT) \
  X(idle_sec, RCLC_PARAMETER_INT)

enum Parameters{
    #define X(a, b) a ##_enum,
    PARAMETERS
    #undef X
    PARAMETERS_SIZE
};

bool initialize_parameters(rclc_parameter_server_t * param_server);
void on_parameter_changed(Parameter * param);

#endif  // MICROROS_PARAMETERS_H_
