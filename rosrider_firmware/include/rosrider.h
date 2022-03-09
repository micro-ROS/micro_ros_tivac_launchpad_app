#ifndef rosrider_h
#define rosrider_h

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <ros.h>
#include <ros/time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"
#include "driverlib/hibernate.h"
#include "driverlib/eeprom.h"

#define PI 3.14159265

int DEFAULT_STATUS = 0;

/* parameters */

// main parameters
uint8_t UPDATE_RATE;
uint8_t ENCODER_PPR;
uint16_t GEAR_RATIO;
float WHEEL_DIA;
float BASE_WIDTH;
float MAIN_AMP_LIMIT;
float MT_AMP_LIMIT;
float BAT_VOLTS_HIGH;
float BAT_VOLTS_LOW;
float MAX_RPM;

// these are imagninary derived from config status
bool LEFT_REVERSE;
bool RIGHT_REVERSE;
bool LEFT_SWAP;
bool RIGHT_SWAP;
bool LEFT_ENC_AB;
bool RIGHT_ENC_AB;
// until here

float LEFT_DEADZONE;
float RIGHT_DEADZONE;
uint16_t MAX_IDLE_SECONDS;

// calculated parameters
uint16_t PULSE_PER_REV;
float WHEEL_CIRCUMFERENCE;
float LINEAR_RPM;
float ANGULAR_RPM;
float TICKS_PER_METER;
float ROUNDS_PER_MINUTE;
float COMMAND_TIMEOUT;

// pid parameters
float left_Kp = 4.8f;
float left_Ki = 3.2f;
float left_Kd = 0.02f;
float right_Kp = 4.8f;
float right_Ki = 3.2f;
float right_Kd = 0.02f;

/* parameters end */

// rtc related
#define SEC_MINUTE 60
#define SEC_HOUR 3600

uint32_t ui32SysClkFreq;
bool eeprom_init = false;
double idle_seconds = 0.0;

// configuration flags
uint8_t config_flags = 48;

// power, motor, system status registers
uint8_t power_status = 0x00;
uint8_t motor_status = 0x00;
uint8_t system_status = 0x00;

char loginfo_buffer[100];

void blink(bool on) {
    if(on) {
       MAP_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
    } else {
       MAP_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x0);
    }
}

void aux_ctrl(bool on) {
    if(on) {
        MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
        power_status |= 0b00000001; // power:bit0 set
    } else {
        MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x0);
        power_status &= 0b11111110; // power:bit0 unset
    }
}

void factory_defaults() {
    // main parameters
    UPDATE_RATE = 10;             // 10,20, or 50hz
    ENCODER_PPR = 12;             // 12 pulses per rev
    GEAR_RATIO = 380;             // 380:1
    WHEEL_DIA = 0.06;             // 6cm
    BASE_WIDTH = 0.1f;            // 10cm
    MAIN_AMP_LIMIT = 3.6f;        // 3.6 amps
    MT_AMP_LIMIT = 1.2f;          // 1.2 amps
    BAT_VOLTS_HIGH = 14.4f;       // 14.4 volts
    BAT_VOLTS_LOW = 6.0f;         // 6.0 volts
    MAX_RPM = 90.0f;              // 90 rpm
    LEFT_DEADZONE = 0.0f;         // 0
    RIGHT_DEADZONE = 0.0f;        // 0
    MAX_IDLE_SECONDS = 1800;      // 1800
}

void init_boolean_parameters() {
    if(config_flags & 0b00000001) { LEFT_REVERSE = true; } else { LEFT_REVERSE = false; }
    if(config_flags & 0b00000010) { RIGHT_REVERSE = true; } else { RIGHT_REVERSE = false; }
    if(config_flags & 0b00000100) { LEFT_SWAP = true; } else { LEFT_SWAP = false; }
    if(config_flags & 0b00001000) { RIGHT_SWAP = true; } else { RIGHT_SWAP = false; }
    if(config_flags & 0b00010000) { LEFT_ENC_AB = true; } else { LEFT_ENC_AB = false; }
    if(config_flags & 0b00100000) { RIGHT_ENC_AB = true; } else { RIGHT_ENC_AB = false; }
}

void recalculate_params() {
    PULSE_PER_REV = GEAR_RATIO * ENCODER_PPR;
    WHEEL_CIRCUMFERENCE = WHEEL_DIA * PI;
    LINEAR_RPM = (1.0f / WHEEL_CIRCUMFERENCE) * 60.0f;
    ANGULAR_RPM = (BASE_WIDTH / (WHEEL_CIRCUMFERENCE * 2.0f)) * 60.0f;
    TICKS_PER_METER = PULSE_PER_REV * (1.0f / WHEEL_CIRCUMFERENCE);
    ROUNDS_PER_MINUTE = (60.0f / (1.0f / UPDATE_RATE)) / PULSE_PER_REV;
    COMMAND_TIMEOUT = (2.0f / UPDATE_RATE);     // notice: two updates, then time out, thus 2.0f
    init_boolean_parameters();
}

// notice: must call init_leds after
void reset_led_bus() {
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);             // configure pf1 as output
    for(int i=0; i<3; i++) {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);      // write 1 to pf1
        MAP_SysCtlDelay(1500UL);                                        // wait 50+ microseconds
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);             // write 0 to pf1
        MAP_SysCtlDelay(1500UL);                                        // wait 50+ microseconds
    }
}

void init_system() {

    // record system clock value after clock initialization
    ui32SysClkFreq = MAP_SysCtlClockGet();

    // Peripheral A, B, C, D, E, F Enable
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // this is a hack to get D7 working. if commented out, encoder will give wrong data.
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    // unlock PF_0
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;

    // reset led bus
    reset_led_bus();

    // PC7 is REDLED
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

    // PB4 is AUXCTL
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);

    // Hibernate module setup
    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_HIBERNATE)) {}

    // determine wake reason
    if(HibernateIsActive()) {
        uint32_t ui32Status= HibernateIntStatus(false);
        if(ui32Status & HIBERNATE_INT_PIN_WAKE) {
            // wake up was due to WAKE pin assertion
        }
        if(ui32Status & HIBERNATE_INT_RTC_MATCH_0) {
            // wake up was due to RTC match register
        }
    } else {
        // cold power-up
    }

    // hibernate setup
	HibernateEnableExpClk(ui32SysClkFreq);
	HibernateGPIORetentionDisable();
    HibernateWakeSet(HIBERNATE_WAKE_PIN);

    // rtc
    HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
    HibernateRTCEnable();
    // TODO: must be able set to a trim value from eeprom
    HibernateRTCTrimSet(0x7FFF);

    // clear hibernate interrupt
    uint32_t ui32Status = HibernateIntStatus(0);
    HibernateIntClear(ui32Status);

    // 50mS delay
	MAP_SysCtlDelay(1333333UL); // 50ms

	// PD0 is USB_BTN
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	// set defaults
	factory_defaults();

    // recalculate parameters
    recalculate_params();

}

/* rtc related */
uint8_t getHours(uint32_t posixTime) {
    uint32_t hours = posixTime / SEC_HOUR;
    return hours % 24;
}

uint8_t getMinutes(uint32_t posixTime) {
    uint32_t minutes = posixTime / SEC_MINUTE;
    return minutes % 60;
}

uint8_t getSeconds(uint32_t posixTime) {
    return posixTime % 60;
}

void setTime(uint32_t posixTime) {
    HibernateRTCSet((getHours(posixTime) * SEC_HOUR) + (getMinutes(posixTime) * SEC_MINUTE) + getSeconds(posixTime));
}

/* rtc related end */

#ifdef __cplusplus
}
#endif

#endif