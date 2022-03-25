#include "TivaQEI.h"

Encoder::Encoder() {
}

void Encoder::init_encoder(uint32_t ui32Base, uint32_t period, bool swap, bool ab) {

    this->ui32Base = ui32Base;

    // setup gpio pins for qei
    if(QEI0_BASE == ui32Base) {

      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

      MAP_GPIOPinConfigure(GPIO_PD6_PHA0);
      MAP_GPIOPinConfigure(GPIO_PD7_PHB0);

      MAP_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    } else if(QEI1_BASE == ui32Base) {

      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

      MAP_GPIOPinConfigure(GPIO_PC5_PHA1);
      MAP_GPIOPinConfigure(GPIO_PC6_PHB1);

      MAP_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

    }

    // disable before config
    MAP_QEIDisable(ui32Base);
    MAP_QEIIntDisable(ui32Base, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // notice: below is ugly code, but readable and it works.
    if(swap) {
        if(ab) {
            MAP_QEIConfigure(ui32Base, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4294967295);
        } else {
            MAP_QEIConfigure(ui32Base, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4294967295);
        }
    } else {
        if(ab) {
            MAP_QEIConfigure(ui32Base, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE), 4294967295);
        } else {
            MAP_QEIConfigure(ui32Base, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE), 4294967295);
        }
    }

    // enable encoder
    MAP_QEIEnable(ui32Base);

    // disable before config
    MAP_QEIVelocityDisable(ui32Base);

    // velocity config
    MAP_QEIVelocityConfigure(ui32Base, QEI_VELDIV_1, period);

    // enable velocity
    MAP_QEIVelocityEnable(ui32Base);

    // set initial position
    MAP_QEIPositionSet(ui32Base, 0);

}

void Encoder::set_encoder_position(uint32_t position) {
  MAP_QEIPositionSet(ui32Base, position);
}

uint32_t Encoder::get_velocity() {
  return MAP_QEIVelocityGet(ui32Base);
}

uint32_t Encoder::get_position() {
  return MAP_QEIPositionGet(ui32Base);
}

int32_t Encoder::get_direction() {
  return MAP_QEIDirectionGet(ui32Base);
}

int16_t Encoder::get_delta() {
  e_position = get_position();
  d_position = e_position - p_position;
  if(d_position < -2147483648) {
    d_position = (2147483648 - p_position) + e_position;
  } else if(d_position > 2147483648) {
    d_position = ((2147483648 - e_position) + p_position) * -1;
  }
  p_position = e_position;
  return d_position;
}
