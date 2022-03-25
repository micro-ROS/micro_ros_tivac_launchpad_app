#ifndef PID_h
#define PID_h

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "motor_enums.h"

class PID {

  public:

    PID(float* current_rpm, float* target_rpm, float* output, float _Kp, float _Ki, float _Kd);

    void set_tunings(float _Kp, float _Ki, float _Kd);
    void set_motor_dir(motor_dir_t dir);

    void update(double dt);

    void reset_pid();

    float get_Kp();
    float get_Ki();
    float get_Kd();

    motor_dir_t get_phase();
    void set_phase(motor_dir_t phase);

  private:

    motor_dir_t pid_phase;

    const int16_t min_out = -1023;
    const int16_t max_out = 1023;

    float Kp;
    float Ki;
    float Kd;

    float *p_current_rpm;
    float *p_target_rpm;
    float *p_output;

    float last_rpm = 0.0;
    float integral_total = 0.0f;

};

#ifdef __cplusplus
}
#endif

#endif