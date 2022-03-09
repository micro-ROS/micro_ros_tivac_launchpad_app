#ifndef ros_leds_h
#define ros_leds_h

#include "rosrider/LedService.h"

#ifdef __cplusplus
extern "C"
{
#endif

// TODO: reset code

#define SPILONG    0xe0
#define SPISHORT   0x40

// holds the led colors, 3ch each, 5 leds
uint8_t led_buffer[15];

// if mask for an index is true, led will be solid, when frequency > 0
bool led_mask[5];

volatile bool leds_ready = false;
volatile bool leds_state = false;
volatile bool timer_disabled = false;

void led_delay() {
    // 50ms
    MAP_SysCtlDelay(1333333UL);
}

// led timer interrupt handler
void Timer1IntHandler(void) {
    leds_ready = true;
    leds_state = !leds_state;
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

// led timer initialization, frequency must be <
void Timer1Init(float frequency) {
    if(frequency < 1.0f) { frequency = 1.0f; }
    if(frequency > 5.0f) { frequency = 5.0f; }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    IntMasterEnable();
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    // notice: frequency is multiplied by two, because we need 2 interrupts to finish 1 cycle
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClkFreq / (frequency * 2));
    TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1IntHandler);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
}

void send_byte(unsigned char b) {
    for(unsigned char bit = 0; bit < 8; bit++) {
      if(b & 0x80) {
        MAP_SSIDataPut(SSI1_BASE, SPILONG);
      } else {
        MAP_SSIDataPut(SSI1_BASE, SPISHORT);
      }
      b <<= 1;
    }
}

void reset_led_buffer() {
    for(uint8_t i=0; i<15; i++) { led_buffer[i] = 0x00; }
}

void reset_led_mask() {
    for(uint8_t i=0; i<5; i++) { led_mask[i] = false; }
}

void send_led_buffer() {
  for(uint8_t i=0; i<5; i++) {
    send_byte(led_buffer[(i*3)+0]);
    send_byte(led_buffer[(i*3)+1]);
    send_byte(led_buffer[(i*3)+2]);
    while(MAP_SSIBusy(SSI1_BASE)) {}
  }
}

void send_blank_buffer() {
  for(uint8_t i=0; i<5; i++) {
    send_byte(0x00);
    send_byte(0x00);
    send_byte(0x00);
    while(MAP_SSIBusy(SSI1_BASE)) {}
  }
}

void send_masked_blank_buffer() {
  for(uint8_t i=0; i<5; i++) {
    if(led_mask[i]) {
        send_byte(led_buffer[(i*3)+0]);
        send_byte(led_buffer[(i*3)+1]);
        send_byte(led_buffer[(i*3)+2]);
    } else {
        send_byte(0x00);
        send_byte(0x00);
        send_byte(0x00);
    }
    while(MAP_SSIBusy(SSI1_BASE)) {}
  }
}

void handle_leds() {
    if(leds_state) {
        send_led_buffer();
    } else if(!timer_disabled) {
        send_masked_blank_buffer();
    }
}

void compose_color(uint32_t color, uint8_t index) {
  // led mask determined by bit0
  led_mask[index/3] = (color & 0x00000001) > 0;
  // red
  led_buffer[index] = (color & 0xFF000000) >> 24;
  // green
  led_buffer[index + 1] = (color & 0x00FF0000) >> 16;
  // blue
  led_buffer[index + 2] = (color & 0x000000FF00) >> 8;
}

void program_leds(uint32_t fl, uint32_t top, uint32_t fr, uint32_t br, uint32_t bl, float frequency) {

    compose_color(fl, 0);
    compose_color(top, 3);
    compose_color(fr, 6);
    compose_color(br, 9);
    compose_color(bl, 12);

    if(frequency==0.0f) {
        timer_disabled = true;
        leds_ready = true;
        // TODO: also have to shut off leds, by sending blank pattern, add this feature then test
    } else {
        // will re-init timer1 for generating timer interrupt two times the frequency given
        timer_disabled = false;
        Timer1Init(frequency);
    }

}

// TODO: led frequency below 1hz
// TODO: mask vs frequency, 0.0 should do something

void init_leds() {

  reset_led_buffer();
  reset_led_mask();

  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

  MAP_GPIOPinConfigure(GPIO_PF2_SSI1CLK);
  //MAP_GPIOPinConfigure(GPIO_PF3_SSI1FSS);
  //MAP_GPIOPinConfigure(GPIO_PF0_SSI1RX);
  MAP_GPIOPinConfigure(GPIO_PF1_SSI1TX);

  //MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2);

  MAP_SSIConfigSetExpClk(SSI1_BASE, ui32SysClkFreq, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 2400000, 8);
  MAP_SSIEnable(SSI1_BASE);

  led_delay();

  send_blank_buffer();

}

// service callback
void leds_cb(const rosrider::LedServiceRequest& req, rosrider::LedServiceResponse& resp) {
    program_leds(req.fl, req.top, req.fr, req.br, req.bl, req.frequency);
    resp.success =  true;
}
ros::ServiceServer<rosrider::LedServiceRequest, rosrider::LedServiceResponse> leds_service("rosrider/led_emitter", &leds_cb);

#ifdef __cplusplus
}
#endif

#endif
