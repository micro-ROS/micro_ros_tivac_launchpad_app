#ifndef rosrider_h
#define rosrider_h

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

#define SEC_MINUTE 60
#define SEC_HOUR 3600

void blink(bool on) {
    if(on) {
       MAP_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
    } else {
       MAP_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x0);
    }
}

// TODO(can): This is not compiling
// void aux_ctrl(bool on) {
//     if(on) {
//         MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
//         power_status |= 0b00000001; // power:bit0 set
//     } else {
//         MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x0);
//         power_status &= 0b11111110; // power:bit0 unset
//     }
// }

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

void init_rosrider_system() {

    // record system clock value after clock initialization
    uint32_t ui32SysClkFreq = MAP_SysCtlClockGet();

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
    // recalculate parameters

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

#endif
