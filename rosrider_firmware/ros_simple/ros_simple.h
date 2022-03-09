#ifndef ros_simple_h
#define ros_simple_h

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

volatile bool data_ready = false;
volatile uint8_t command_timeout = 0;

bool blink_state = false;
float current_rpm_left, current_rpm_right;

// delayed executor system
uint8_t system_command = 0;
double system_delay = 0.0;

ros::NodeHandle nh;
ros::Time current_time;
ros::Time marked_time;

std_msgs::Float64 left_wheel_state;
std_msgs::Float64 right_wheel_state;

std_msgs::Int32 left_wheel_position;
std_msgs::Int32 right_wheel_position;

ros::Publisher left_wheel_velocity_pub("left_wheel/state", &left_wheel_state);
ros::Publisher right_wheel_velocity_pub("right_wheel/state", &left_wheel_state);

ros::Publisher left_wheel_position_pub("left_wheel/position", &left_wheel_position);
ros::Publisher right_wheel_position_pub("right_wheel/position", &left_wheel_position);

void hibernate_request() {
    blink(false);
    aux_ctrl(false);
    HibernateRequest();
}

void soft_fuse_reset(void) {
    aux_ctrl(false);
    power_status = 0x00;
    idle_seconds = 0.0;
}

void reset_state(void) {
    // IDLE0
    idle_seconds = 0.0;
    reset_led_buffer();
    reset_led_mask();
}

uint32_t write_EEPROM_DEFAULT_STATUS(void) {
    uint32_t WRITE_RESULT = EEPROMProgram((uint32_t *)&DEFAULT_STATUS, 0x00, sizeof(DEFAULT_STATUS));
    if(WRITE_RESULT!=0) { system_status |= 0b00000010; } else { system_status &= 0b11111101; } // system:bit1
    return WRITE_RESULT;
}

void read_EEPROM_DEFAULT_STATUS(void) {
    EEPROMRead((uint32_t *)&DEFAULT_STATUS, 0x00, sizeof(DEFAULT_STATUS));
}

uint32_t write_EEPROM_PARAMETERS(void) {

    uint32_t WRITE_RESULT = 0;

    WRITE_RESULT += EEPROMProgram((uint32_t *)&config_flags, 0x04, sizeof(config_flags));
    WRITE_RESULT += EEPROMProgram((uint32_t *)&UPDATE_RATE, 0x08, sizeof(UPDATE_RATE));
    WRITE_RESULT += EEPROMProgram((uint32_t *)&ENCODER_PPR, 0x0C, sizeof(ENCODER_PPR));
    WRITE_RESULT += EEPROMProgram((uint32_t *)&GEAR_RATIO, 0x10, sizeof(GEAR_RATIO));

    pui32Data[0] = ((union conv32){.f32 = WHEEL_DIA}).u32;
    WRITE_RESULT += EEPROMProgram(pui32Data, 0x14, sizeof(pui32Data));

    pui32Data[0] = ((union conv32){.f32 = BASE_WIDTH}).u32;
    WRITE_RESULT += EEPROMProgram(pui32Data, 0x18, sizeof(pui32Data));

    pui32Data[0] = ((union conv32){.f32 = MAIN_AMP_LIMIT}).u32;
    WRITE_RESULT += EEPROMProgram(pui32Data, 0x1C, sizeof(pui32Data));

    pui32Data[0] = ((union conv32){.f32 = MT_AMP_LIMIT}).u32;
    WRITE_RESULT += EEPROMProgram(pui32Data, 0x20, sizeof(pui32Data));

    pui32Data[0] = ((union conv32){.f32 = BAT_VOLTS_HIGH}).u32;
    WRITE_RESULT += EEPROMProgram(pui32Data, 0x24, sizeof(pui32Data));

    pui32Data[0] = ((union conv32){.f32 = BAT_VOLTS_LOW}).u32;
    WRITE_RESULT += EEPROMProgram(pui32Data, 0x28, sizeof(pui32Data));

    pui32Data[0] = ((union conv32){.f32 = MAX_RPM}).u32;
    WRITE_RESULT += EEPROMProgram(pui32Data, 0x2C, sizeof(pui32Data));

    WRITE_RESULT += EEPROMProgram((uint32_t *)&MAX_IDLE_SECONDS, 0x30, sizeof(MAX_IDLE_SECONDS));

    if(WRITE_RESULT!=0) { system_status |= 0b00000100; } else { system_status &= 0b11111011; } // system:bit2

    return WRITE_RESULT;

}

void read_EEPROM_PARAMETERS(void) {

    EEPROMRead((uint32_t *)&config_flags, 0x04, sizeof(config_flags));
    EEPROMRead((uint32_t *)&UPDATE_RATE, 0x08, sizeof(UPDATE_RATE));
    EEPROMRead((uint32_t *)&ENCODER_PPR, 0x0C, sizeof(ENCODER_PPR));
    EEPROMRead((uint32_t *)&GEAR_RATIO, 0x10, sizeof(GEAR_RATIO));

    EEPROMRead(pui32Read, 0x14, sizeof(pui32Read));
    WHEEL_DIA = ((union conv32){.u32 = pui32Read[0]}).f32;

    EEPROMRead(pui32Read, 0x18, sizeof(pui32Read));
    BASE_WIDTH = ((union conv32){.u32 = pui32Read[0]}).f32;

    EEPROMRead(pui32Read, 0x1C, sizeof(pui32Read));
    MAIN_AMP_LIMIT = ((union conv32){.u32 = pui32Read[0]}).f32;

    EEPROMRead(pui32Read, 0x20, sizeof(pui32Read));
    MT_AMP_LIMIT = ((union conv32){.u32 = pui32Read[0]}).f32;

    EEPROMRead(pui32Read, 0x24, sizeof(pui32Read));
    BAT_VOLTS_HIGH = ((union conv32){.u32 = pui32Read[0]}).f32;

    EEPROMRead(pui32Read, 0x28, sizeof(pui32Read));
    BAT_VOLTS_LOW = ((union conv32){.u32 = pui32Read[0]}).f32;

    EEPROMRead(pui32Read, 0x2C, sizeof(pui32Read));
    MAX_RPM = ((union conv32){.u32 = pui32Read[0]}).f32;

    EEPROMRead((uint32_t *)&MAX_IDLE_SECONDS, 0x30, sizeof(MAX_IDLE_SECONDS));

}

void print_rtc_time() {
    uint32_t posixTime = HibernateRTCGet();
    sprintf(loginfo_buffer, "RTC Clock = %02d:%02d:%02d", getHours(posixTime), getMinutes(posixTime), getSeconds(posixTime));
    spin_print(nh);
}

// service callback

void sysctrl_cb(const rosrider::SysCtrlServiceRequest& req, rosrider::SysCtrlServiceResponse& resp) {

    // notice: not every operation returns success
    resp.success =  true;

    switch(req.command) {

      case 0:  // software fuse reset
        soft_fuse_reset();
        break;
      case 1:  // hardware reset
        system_command = 1;
        break;
      case 2: // encoder reset
        zero_encoders();
        break;
      case 3: // hibernate
        system_command = 3;
        break;

      case 20: // motor left, mode 2, on
        left_mode2(true);
        break;
      case 21: // motor left, mode 2, off
        left_mode2(false);
        break;
      case 22: // motor right, mode 2, on
        right_mode2(true);
        break;
      case 23: // motor right, node 2, off
        right_mode2(false);
        break;

      case 24: // motor left, mode 1, on
        left_mode1(true);
        break;
      case 25: // motor left, mode 1, off
        left_mode1(false);
        break;
      case 26: // motor right, mode 1, on
        right_mode1(true);
        break;
      case 27: // motor right, mode 1, off
        right_mode1(false);
        break;

      case 30: // motor direction left: forward
        left_phase(FORWARD);
        break;
      case 31: // motor direction left: reverse
        left_phase(REVERSE);
        break;
      case 32: // motor direction right: forward
        right_phase(FORWARD);
        break;
      case 33: // motor direction right: reverse
        right_phase(REVERSE);
        break;

      case 40: // both mode1 on: brakes on
        left_mode1(true);
        right_mode1(true);
        break;
      case 41: // both mode1 off: brakes off
        left_mode1(false);
        right_mode1(false);
        break;
      case 42: // both mode2 on
        left_mode2(true);
        right_mode2(true);
        break;
      case 43: // both mode2 off
        left_mode2(false);
        right_mode2(false);
        break;
      case 44: // both phase forward
        left_phase(FORWARD);
        right_phase(FORWARD);
        break;
      case 45: // both phase reverse
        left_phase(REVERSE);
        right_phase(REVERSE);
        break;

      case 50:
        aux_ctrl(true);
        break;
      case 51:
        aux_ctrl(false);
        break;

      case 80: // default_status = 0, device will request parameters
        DEFAULT_STATUS = 0;
        if(eeprom_init) {
            if(write_EEPROM_DEFAULT_STATUS()==0) {
                sprintf(loginfo_buffer, "setting default_status=%d", DEFAULT_STATUS);
            } else {
                resp.success =  false;
                sprintf(loginfo_buffer, "setting default_status=%d failed", DEFAULT_STATUS);
            }
            spin_print(nh);
        } else {
            resp.success =  false;
            sprintf(loginfo_buffer, "EEPROM is not initialized");
        }
        break;
      case 81: // write paramters to eeprom, set default_status = 1
        DEFAULT_STATUS = 1;
        if(eeprom_init) {
            if(write_EEPROM_PARAMETERS()==0) {
                sprintf(loginfo_buffer, "writing params to eeprom", DEFAULT_STATUS);
                spin_print(nh);
                if(write_EEPROM_DEFAULT_STATUS()==0) {
                    sprintf(loginfo_buffer, "setting default_status=%d", DEFAULT_STATUS);
                } else {
                    resp.success =  false;
                    sprintf(loginfo_buffer, "setting default_status=%d failed", DEFAULT_STATUS);
                }
                spin_print(nh);
            } else {
                resp.success =  false;
                sprintf(loginfo_buffer, "writing params to eeprom failed", DEFAULT_STATUS);
                spin_print(nh);
            }
        } else {
            resp.success =  false;
            sprintf(loginfo_buffer, "EEPROM is not initialized");
        }
        break;
      case 82: // print parameters
        print_default_status(nh);
        print_parameters(nh);
        print_system_flags(nh);
        break;

      case 90: // print RTC time
        // TODO: DOC: add to documentation
        // TODO: BUG: sometimes displays 00:00:00 after couple of tries
        print_rtc_time();
        break;
      case 91: // set RTC time
        // TODO: DOC: add to documentation
        // nh.now() returns mcu time as double
        // tuncating time within 0 to 1.0 secs, which acts like a 1s delay
        setTime((uint32_t) nh.now().toSec());
        break;

      default:
        break;
    }

    // TODO: AUDIT: auto sleep functions
    // TODO: FEATURE: calibration of current sense biases
    // TODO: FATURE: deadzone audit, and add some sort of calib
    // TODO: pwm frequency test for the blue motors

    // TODO: first do catcher for parameters from param file
    // TODO: second change mt left right
    // TODO: change in eeprom, read write



}
ros::ServiceServer<rosrider::SysCtrlServiceRequest, rosrider::SysCtrlServiceResponse> sysctrl_service("rosrider/sysctl", &sysctrl_cb);

void left_wheel_control_effort_cb(const std_msgs::Float64& msg) {
  pwm_left = (int16_t) msg.data;
  if(pwm_left >= 0.0) {
    left_phase(FORWARD);
  } else if(pwm_left < 0.0) {
    left_phase(REVERSE);
  }
  left_pwm(abs(pwm_left));
  command_timeout = 0;
}
ros::Subscriber<std_msgs::Float64> left_wheel_control_effort_sub("left_wheel/control_effort", &left_wheel_control_effort_cb);

void right_wheel_control_effort_cb(const std_msgs::Float64& msg) {
  pwm_right = (int16_t) msg.data;
  if(pwm_right >= 0.0) {
    right_phase(FORWARD);
  } else if(pwm_right < 0.0) {
    right_phase(REVERSE);
  }
  right_pwm(abs(pwm_right));
  command_timeout = 0;
}
ros::Subscriber<std_msgs::Float64> right_wheel_control_effort_sub("right_wheel/control_effort", &right_wheel_control_effort_cb);

// main timer interrupt handler
void Timer0IntHandler(void) {
    data_ready = true;
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

// main timer initialization
void Timer0Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq / UPDATE_RATE);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// initializes eeprom
void initialize_eeprom() {

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    // Try 3 times to enable eeprom, if not fail safe
    for(int i=0; i<3; i++) {
        if(EEPROMInit()==EEPROM_INIT_OK) { eeprom_init = true; break; }
        MAP_SysCtlDelay(13333UL);
    }

    // if eeprom is init successfully, read default status, if not, set default_status = 0
    if(eeprom_init) {
        read_EEPROM_DEFAULT_STATUS();
        system_status &= 0b11111110; // system:bit0 unset
    } else {
        DEFAULT_STATUS = 0;
        system_status |= 0b00000001; // system:bit0 set
    }

    // failsafe in case default_status misread from 0x00
    if(DEFAULT_STATUS < 0) { DEFAULT_STATUS = 0; }

}

// this method is executed when power fail is detected
void self_power_fail() {

    uint8_t blink_counter = 0;
    reset_led_bus();

    while(1) {

        // TODO: DOC: the behaivor: once we are here, usb is not working, system is locked up.
        // TODO: DOC: unit will blink 100 times, then go in hibernate mode

        // if user_button is pressed, unit goes on hibernate mode directly
        if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)==0) { hibernate_request(); }

        if(blink_counter >= 100) {
            hibernate_request();
        } else {
            for(uint8_t i=0; i<3; i++) {
                blink(true);
                MAP_SysCtlDelay(6000000UL);
                blink(false);
                MAP_SysCtlDelay(6000000UL);
            }
            for(uint8_t i=0; i<3; i++) {
                blink(true);
                MAP_SysCtlDelay(2000000UL);
                blink(false);
                MAP_SysCtlDelay(2000000UL);
            }
            MAP_SysCtlDelay(2000000UL);
            blink_counter += 1;
        }


    }
}

#endif