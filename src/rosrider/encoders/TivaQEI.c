#include "TivaQEI.h"

void init_encoder(encoder_t * this, uint32_t ui32Base_in, uint32_t period, bool swap, bool ab) {

    this->ui32Base = ui32Base_in;

    // setup gpio pins for qei
    if(QEI0_BASE == this->ui32Base) {

      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

      MAP_GPIOPinConfigure(GPIO_PD6_PHA0);
      MAP_GPIOPinConfigure(GPIO_PD7_PHB0);

      MAP_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    } else if(QEI1_BASE == this->ui32Base) {

      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

      MAP_GPIOPinConfigure(GPIO_PC5_PHA1);
      MAP_GPIOPinConfigure(GPIO_PC6_PHB1);

      MAP_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

    }

    // disable before config
    MAP_QEIDisable(this->ui32Base);
    MAP_QEIIntDisable(this->ui32Base, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // notice: below is ugly code, but readable and it works.
    if(swap) {
        if(ab) {
            MAP_QEIConfigure(this->ui32Base, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4294967295);
        } else {
            MAP_QEIConfigure(this->ui32Base, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4294967295);
        }
    } else {
        if(ab) {
            MAP_QEIConfigure(this->ui32Base, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE), 4294967295);
        } else {
            MAP_QEIConfigure(this->ui32Base, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE), 4294967295);
        }
    }

    // enable encoder
    MAP_QEIEnable(this->ui32Base);

    // disable before config
    MAP_QEIVelocityDisable(this->ui32Base);

    // velocity config
    MAP_QEIVelocityConfigure(this->ui32Base, QEI_VELDIV_1, period);

    // enable velocity
    MAP_QEIVelocityEnable(this->ui32Base);

    // set initial position
    MAP_QEIPositionSet(this->ui32Base, 0);

}

void set_encoder_position(encoder_t * this, uint32_t position) {
  MAP_QEIPositionSet(this->ui32Base, position);
}

uint32_t get_velocity(encoder_t * this) {
  return MAP_QEIVelocityGet(this->ui32Base);
}

uint32_t get_position(encoder_t * this) {
  return MAP_QEIPositionGet(this->ui32Base);
}

int32_t get_direction(encoder_t * this) {
  return MAP_QEIDirectionGet(this->ui32Base);
}

int16_t get_delta(encoder_t * this) {
  this->e_position = get_position(this);
  this->d_position = this->e_position - this->p_position;
  if(this->d_position < -2147483648) {
    this->d_position = (2147483648 - this->p_position) + this->e_position;
  } else if(this->d_position > 2147483648) {
    this->d_position = ((2147483648 - this->e_position) + this->p_position) * -1;
  }
  this->p_position = this->e_position;
  return this->d_position;
}
