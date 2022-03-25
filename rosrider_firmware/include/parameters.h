#ifndef PARAMETERS_H
#define PARAMETERS_H

#define PARAM_SIZE 14
bool parameters[PARAM_SIZE];

uint32_t pui32Data[1];
uint32_t pui32Read[1];

union conv32 {
    uint32_t u32;
    float f32;
};

// void spin_print(ros::NodeHandle &nh) {
//     nh.loginfo(loginfo_buffer);
//     nh.spinOnce();
//     nh.getHardware()->delay(10);
// }

// void spin_delay(ros::NodeHandle &nh) {
//     nh.spinOnce();
//     nh.getHardware()->delay(10);
// }

// notice: do not change this.
// void print_bool(char *str, bool value) {
//     if(value) { sprintf(loginfo_buffer, str, "true"); } else { sprintf(loginfo_buffer, str, "false"); }
// }

// void handle_parameters(ros::NodeHandle &nh) {

//     for(int i=0; i<PARAM_SIZE; i++) { parameters[i] = false; }

//     bool success = true;

//     parameters[0] = nh.getParam("rosrider/config_flags", (int*) &config_flags, 1, 1000);
//     spin_delay(nh);

//     parameters[1] = nh.getParam("rosrider/update_rate", (int*) &UPDATE_RATE, 1, 1000);
//     spin_delay(nh);

//     if(!(UPDATE_RATE==10 || UPDATE_RATE==20 || UPDATE_RATE==50)) { UPDATE_RATE = 10; } // if update rate is not 10, 20, or 50, default to 10

//     parameters[2] = nh.getParam("rosrider/encoder_ppr", (int*) &ENCODER_PPR, 1, 1000);
//     spin_delay(nh);

//     parameters[3] = nh.getParam("rosrider/gear_ratio", (int*) &GEAR_RATIO, 1, 1000);
//     spin_delay(nh);

//     parameters[4] = nh.getParam("rosrider/wheel_dia", (float*) &WHEEL_DIA, 1, 1000);
//     spin_delay(nh);

//     parameters[5] = nh.getParam("rosrider/base_width", (float*) &BASE_WIDTH, 1, 1000);
//     spin_delay(nh);

//     parameters[6] = nh.getParam("rosrider/main_amp_limit", (float*) &MAIN_AMP_LIMIT, 1, 1000);
//     spin_delay(nh);

//     parameters[7] = nh.getParam("rosrider/left_wheel/mt_amp_limit", (float*) &MT_AMP_LIMIT, 1, 1000);
//     spin_delay(nh);

//     parameters[8] = nh.getParam("rosrider/bat_volts_high", (float*) &BAT_VOLTS_HIGH, 1, 1000);
//     spin_delay(nh);

//     parameters[9] = nh.getParam("rosrider/bat_volts_low", (float*) &BAT_VOLTS_LOW, 1, 1000);
//     spin_delay(nh);

//     parameters[10] =  nh.getParam("rosrider/max_rpm", (float*) &MAX_RPM, 1, 1000);
//     spin_delay(nh);

//     // TODO: DOC: deadzone is now float
//     parameters[11] = nh.getParam("rosrider/left_wheel/deadzone", (float*) &LEFT_DEADZONE, 1, 1000);
//     spin_delay(nh);

//     parameters[12] = nh.getParam("rosrider/right_wheel/deadzone", (float*) &RIGHT_DEADZONE, 1, 1000);
//     spin_delay(nh);

//     parameters[13] = nh.getParam("rosrider/max_idle_seconds", (int*) &MAX_IDLE_SECONDS, 1, 1000);
//     spin_delay(nh);

//     // scan for parameters array to find if any parameter failed
//     for(int i=0; i<PARAM_SIZE; i++) { if(parameters[i]==false) { success = false; break; } }

//     if(!success) {
//         system_status |= 0b10000000; // system:bit7 set
//         sprintf(loginfo_buffer, "Loading of at least one parameter failed. drivers disabled.");
//         spin_print(nh);
//     } else {
//         system_status &= 0b01111111; // system:bit7 unset
//     }

//     // notice parameters are recalculated whether success or not.
//     recalculate_params();
// }

// void print_default_status(ros::NodeHandle &nh) {
//     // print default_status
//     sprintf(loginfo_buffer, "default_status: %d", DEFAULT_STATUS);
//     spin_print(nh);
// }

// void print_parameters(ros::NodeHandle &nh) {

//     // print parameters
//     sprintf(loginfo_buffer, "update_rate: %d", UPDATE_RATE);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "encoder_ppr: %d", ENCODER_PPR);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "gear_ratio: %d", GEAR_RATIO);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "wheel_dia: %.3f", WHEEL_DIA);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "base_width: %.3f", BASE_WIDTH);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "main_amp_limit: %.2f", MAIN_AMP_LIMIT);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "mt_amp_limit: %.2f", MT_AMP_LIMIT);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "bat_volts_high: %.1f", BAT_VOLTS_HIGH);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "bat_volts_low: %.1f", BAT_VOLTS_LOW);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "max_rpm: %.1f", MAX_RPM);
//     spin_print(nh);

//     print_bool("left_swap: %s", LEFT_SWAP);
//     spin_print(nh);

//     print_bool("right_swap: %s", RIGHT_SWAP);
//     spin_print(nh);

//     print_bool("left_reverse: %s", LEFT_REVERSE);
//     spin_print(nh);

//     print_bool("right_reverse: %s", RIGHT_REVERSE);
//     spin_print(nh);

//     // TODO: these parameters are not depicted in config file nor documentation
//     print_bool("left_enc_ab: %s", LEFT_ENC_AB);
//     spin_print(nh);

//     print_bool("right_enc_ab: %s", RIGHT_ENC_AB);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "left_deadzone: %.2f", LEFT_DEADZONE);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "right_deadzone: %.2f", RIGHT_DEADZONE);
//     spin_print(nh);

//     // TODO: move all left/right parameters under /left_wheel/ /ritghtwheel

//     sprintf(loginfo_buffer, "max_idle_seconds: %d", MAX_IDLE_SECONDS);
//     spin_print(nh);

// }

// void print_system_flags(ros::NodeHandle &nh) {

//     // print statuses
//     sprintf(loginfo_buffer, "config_flags: %d", config_flags);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "power_status: %d", power_status);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "motor_status: %d", motor_status);
//     spin_print(nh);

//     sprintf(loginfo_buffer, "system_status: %d", system_status);
//     spin_print(nh);

// }

#endif