#include "rosrider.h"
#include "ros_encoders.h"
#include "ros_motors.h"
#include "ros_leds.h"
#include "ros_monitor.h"
#include "parameters.h"
#include "rosrider/SysCtrlService.h"
#include "ros_simple.h"

int main(void) {

    // initialize tiva-c @ 80mhz
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // do not change this order
    init_system();
    init_I2C2();
    init_monitor();

    // self power diagnostics
    self_power_diagnostics();

    // do not start system, and go to hibernate after 100 seconds
    if(power_status > 1) { self_power_fail(); }

    // initialize eeprom
    initialize_eeprom();

    // if default_status > 0, read eeprom parameters
    if(DEFAULT_STATUS > 0) {
        read_EEPROM_PARAMETERS();
        recalculate_params();
    }

    // start initializing system
    init_motors(LEFT_REVERSE, RIGHT_REVERSE);
    init_encoders(ui32SysClkFreq / UPDATE_RATE, LEFT_SWAP, RIGHT_SWAP, LEFT_ENC_AB, RIGHT_ENC_AB);

    // init_leds must be last, or it alters pwm characteristics
    init_leds();

    // main timer init, with default update rate
    Timer0Init();

    // led timer init @ 1.0Hz
    Timer1Init(1.0f);

    // init node
    nh.initNode();

    // do not change this order
    nh.advertiseService<rosrider::SysCtrlServiceRequest, rosrider::SysCtrlServiceResponse>(sysctrl_service);
    nh.advertiseService<rosrider::LedServiceRequest, rosrider::LedServiceResponse>(leds_service);

    // publishers
    nh.advertise(left_wheel_velocity_pub);
    nh.advertise(right_wheel_velocity_pub);
    nh.advertise(left_wheel_position_pub);
    nh.advertise(right_wheel_position_pub);
    nh.advertise(diagnostics_pub);

    // subscribers
    nh.subscribe(left_wheel_control_effort_sub);
    nh.subscribe(right_wheel_control_effort_sub);

    // Wait for connection to establish
    while(!nh.connected()) {

      // read USB_BTN, and hibernate request if button pressed
      if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)==0) { hibernate_request(); }

      // IDLE+
      if(MAX_IDLE_SECONDS > 0) { idle_seconds += 0.01; }

      // after max_idle_seconds, hibernate
      if((MAX_IDLE_SECONDS > 0) && (idle_seconds > MAX_IDLE_SECONDS)) { hibernate_request(); }

      nh.spinOnce();
      nh.getHardware()->delay(10);

      // flip led
      blink_state = !blink_state;
      blink(blink_state);

    }

    bool nh_prev_state = false;
    bool nh_connected = false;

    // IDLE0
    idle_seconds = 0.0f;

    marked_time = nh.now();

    while(1) {

        nh_connected = nh.connected();

        if(nh_connected && !nh_prev_state) {

            nh_prev_state = true;

            print_default_status(nh);

            if(DEFAULT_STATUS < 1) {

                // hard request parameters
                handle_parameters(nh);

                print_parameters(nh);

                // reinit timer
                Timer0Init();

                // set motor defaults
                left_rev(LEFT_REVERSE);
                right_rev(RIGHT_REVERSE);

                // reinit encoders with update rate, and swap
                init_encoders(ui32SysClkFreq / UPDATE_RATE, LEFT_SWAP, RIGHT_SWAP, LEFT_ENC_AB, RIGHT_ENC_AB);

            } else {

                // using parameters from eeprom
                recalculate_params();
                print_parameters(nh);

            }

            // print system flags
            print_system_flags(nh);

            // print rtc time
            print_rtc_time();

            // resets state after connect/disconnect
            reset_state();

            nh.spinOnce();
            nh.getHardware()->delay(10);
            continue;
        }

        // after disconnect is detected, this will execute only once
        if(!nh_connected && nh_prev_state) {

          nh_prev_state = false;

          // resets state after connect/disconnect
          reset_state();

          nh.spinOnce();
          nh.getHardware()->delay(10);
          continue;
        }

        // this triggers at system update speed
        if(data_ready) {

            current_time = nh.now();

            double dt = current_time.toSec() - marked_time.toSec();
            marked_time = current_time;

            // stop motors if command timeout > 3
            if(command_timeout > 3) {
                left_pwm(0);
                right_pwm(0);
            }
            command_timeout++;


            // measure encoders
            measure_encoders();

            // read current sensor, and check if power is witin configured nominal
            update_ina219();
            power_check();

            if(nh_connected) {

                // notice to get signed velocity, we must multiply vel by encdir
                int32_t c_left = vel_left * encdir_left;
                int32_t c_right = vel_right * encdir_right;
                left_wheel_state.data = c_left * ROUNDS_PER_MINUTE;
                right_wheel_state.data = c_right * ROUNDS_PER_MINUTE;

                // publish encoder velocity
                left_wheel_velocity_pub.publish(&left_wheel_state);
                right_wheel_velocity_pub.publish(&right_wheel_state);

                // get encoder position
                left_wheel_position.data = (int32_t) pos_left;
                right_wheel_position.data = (int32_t) pos_right;

                // publish encoder position
                left_wheel_position_pub.publish(&left_wheel_position);
                right_wheel_position_pub.publish(&right_wheel_position);

                // send power diagnostics data
                send_diagnostics_msg(dt);

                // IDLE0
                idle_seconds = 0.0;

                // flip led
                blink_state = !blink_state;
                blink(blink_state);

            } else {
                // IDLE+
                if(MAX_IDLE_SECONDS > 0) { idle_seconds += dt; }
                // led off if disconnected
                blink(false);
            }

            nh.spinOnce();

            // delayed executor for hard reset, and hibernate
            if(system_command > 0) {
                if(system_delay < 1.0) { system_delay += dt; } else {
                    switch(system_command) {
                        case 1:
                            SysCtlReset();
                            break;
                        case 3:
                            hibernate_request();
                            break;
                        default:
                            system_delay = 0.0;
                            break;
                    }
                }
            }

            // after max_idle_seconds, hibernate
            if((MAX_IDLE_SECONDS > 0) && (idle_seconds > MAX_IDLE_SECONDS)) { hibernate_request(); }

            data_ready = false;

        } // data_ready end

        // notice: the mcu will continiously execute from here, to while end

        // handle leds, triggered by led timer
        if(leds_ready) {
            handle_leds();
            leds_ready = false;
        }

        // read USB_BTN, and hibernate request if button pressed
        if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)==0) { hibernate_request(); }

        // measure adc voltages for cs_left and cs_right
        update_monitor();

    } // while end

}