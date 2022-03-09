#ifndef TivaQEI_h
#define TivaQEI_h

#define GPIO_PD6_PHA0           0x00031806
#define GPIO_PD7_PHB0           0x00031C06

#define GPIO_PC5_PHA1           0x00021406
#define GPIO_PC6_PHB1           0x00021806

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"


typedef struct {
  uint32_t ui32Base;
  uint32_t e_position;
  uint32_t p_position;
  int16_t d_position;
} encoder_t;

void init_encoder(encoder_t * encoder, uint32_t ui32Base, uint32_t period, bool swap, bool ab);
void set_encoder_position(encoder_t * encoder, uint32_t position);
uint32_t get_position(encoder_t * encoder);
uint32_t get_velocity(encoder_t * encoder);
int32_t get_direction(encoder_t * encoder);
int16_t get_delta(encoder_t * encoder);

#endif
