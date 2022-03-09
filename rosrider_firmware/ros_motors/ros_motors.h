#ifndef ros_motors_h
#define ros_motors_h

#ifdef __cplusplus
extern "C"
{
#endif

#include "motor_enums.h"

#define PWM_FREQUENCY 400

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;

bool left_reverse = false;
bool right_reverse = false;

int16_t pwm_left;
int16_t pwm_right;

// DRV1 is LEFT, DRV2 is RIGHT

void left_rev(bool reverse) {
    left_reverse = reverse;
}

void right_rev(bool reverse) {
    right_reverse = reverse;
}

void left_phase(motor_dir_t dir) {
  if(REVERSE == dir) {
    if(!left_reverse) { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x0); } else { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_4); }
    motor_status &= 0b11101111; // motor:bit4 unset
  } else if(FORWARD == dir) {
    if(!left_reverse) { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2); } else { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x0); }
    motor_status |= 0b00010000; // motor:bit4 set
  }
}

void right_phase(motor_dir_t dir) {
  if(REVERSE == dir) {
    if(!right_reverse) { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x0); } else { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3); }
    motor_status &= 0b11011111; // motor:bit5 unset
  } else if(FORWARD == dir) {
    if(!right_reverse) { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3); } else { MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x0); }
    motor_status |= 0b00100000; // motor:bit5 set
  }
}

void left_mode2(bool mode2) {
   if(mode2) {
     MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
     motor_status |= 0b00000100; // motor:bit2 set
   } else {
     MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x0);
     motor_status &= 0b11111011; // motor:bit2 unset
   }
}

void right_mode2(bool mode2) {
   if(mode2) {
      MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
      motor_status |= 0b00001000; // motor:bit3 set
   } else {
      MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
      motor_status &= 0b11110111; // motor:bit3 unset
   }
}

void left_mode1(bool mode1) {
  if(mode1) {
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    motor_status |= 0b00000001; // motor:bit0 set
  } else {
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x0);
    motor_status &= 0b11111110; // motor:bit0 unset
  }
}

void right_mode1(bool mode1) {
  if(mode1) {
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
    motor_status |= 0b00000010; // motor:bit1 set
  } else {
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x0);
    motor_status &= 0b11111101; // motor:bit1 unset
  }
}

// notice: && is a question. & is an operation.

void left_pwm(uint16_t pwm) {

  if((power_status > 1 || system_status >= 128)) { pwm = 0; }

  if(pwm > 0) { pwm = (LEFT_DEADZONE * 1023.0f) + (pwm * (1.0f - LEFT_DEADZONE)); }

  // notice unable to set full pwm pattern
  if(pwm > 1023) { pwm = 1023; }

  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm * ui32Load / 1024);

}

void right_pwm(uint16_t pwm) {

  if((power_status > 1 || system_status >= 128)) { pwm = 0; }

  if(pwm > 0) { pwm = (RIGHT_DEADZONE * 1023.0f) + (pwm * (1.0f - RIGHT_DEADZONE)); }

  // notice unable to set full pwm pattern
  if(pwm > 1023) { pwm = 1023; }

  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm * ui32Load / 1024);

}

void init_motors(bool _left_rev, bool _right_rev) {

  // REV1E, DRV1 is LEFT, DRV2 is RIGHT

  // notice always use functions to set anything to do with motor_status
  // config status is read only

  // set reverse
  left_rev(_left_rev);
  right_rev(_right_rev);

  // DRV1_ENABLE PB6

  // DRV1_PHASE  PE2
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);

  // DRV1_MODE1  PB5
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

  // DRV1_MODE2  PD1
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);

  // DRV2_ENABLE PB7

  // DRV2_PHASE  PE3
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);

  // DRV2_MODE1  PF4
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);

  // DRV2_MODE2  PD2
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);

  // set pwm frequency
  MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
  ui32PWMClock = ui32SysClkFreq / 8;
  ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  MAP_GPIOPinConfigure(GPIO_PB6_M0PWM0);
  MAP_GPIOPinConfigure(GPIO_PB7_M0PWM1);
  MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
  MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);

  MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
  MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);

  MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

  // send zero pwm
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);

  // set brakes off
  left_mode1(false);
  right_mode1(false);

  // set direction forward
  left_phase(FORWARD);
  right_phase(FORWARD);

}

#ifdef __cplusplus
}
#endif

#endif