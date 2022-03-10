#include "./microros_parameters.h"

#include "driverlib/rom_map.h"
#include "driverlib/eeprom.h"

union conv32 {
    uint32_t u32;
    float f32;
};

extern float WHEEL_DIA;
extern bool eeprom_init;;

void wheel_dia_callback(Parameter * param){
  WHEEL_DIA = (float) param->value.double_value;

  if(eeprom_init) {
    uint32_t pui32Data = ((union conv32){.f32 = WHEEL_DIA}).u32;
    // THIS HALTS THE SYSTEM. IS SAFE TO CALL EEPROMProgram DIRECTLY?
    // EEPROMProgram(pui32Data, 0x14, sizeof(pui32Data));
  }

  recalculate_params();
}

void update_rate_ms_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void motor_polarity_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void enc_direction_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void enc_phase_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void enc_ppr_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void base_width_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void motor_deadzone_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void main_current_limit_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void motor_current_limit_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void battery_volts_low_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void battery_volts_high_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void max_rpm_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }
void idle_sec_callback(Parameter * param){ /* TODO: Do something with the parameter value */ }

bool initialize_parameters(rclc_parameter_server_t * param_server)
{
  bool ret = true;

  #define X(name, type) rclc_add_parameter(param_server, #name, type);
  PARAMETERS
  #undef X

  // Init defaults
  rclc_parameter_set_double(param_server, "wheel_dia", (double) WHEEL_DIA);

  return ret;
}

void on_parameter_changed(Parameter * param)
{
  #define X(a, b) if (strcmp(param->name.data, #a) == 0 && param->value.type == b) {a ## _callback(param);}
  PARAMETERS
  #undef X
}
