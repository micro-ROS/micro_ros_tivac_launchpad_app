#include "rosrider.h"
#include "ros_leds.h"

ros::NodeHandle nh;
ros::Time current_time;
ros::Time marked_time;

int main(void) {

    // initialize tiva-c @ 80mhz
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    init_system();
    init_leds();

    // led timer init @ 1.0Hz
    Timer1Init(1.0f);

    nh.initNode();

    nh.advertiseService<rosrider::LedServiceRequest, rosrider::LedServiceResponse>(leds_service);

    while(1) {

      current_time = nh.now();

      // do processing here.

      if(leds_ready) {
          handle_leds();
          leds_ready = false;
      }

      nh.spinOnce();
      nh.getHardware()->delay(10);
      marked_time = current_time;

    }

}


