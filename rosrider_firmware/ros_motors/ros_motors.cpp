#include "rosrider.h"
#include <std_msgs/Int16.h>
#include "ros_motors.h"

ros::NodeHandle nh;
ros::Time current_time;
ros::Time marked_time;

void pwm_left_cb(const std_msgs::Int16& pwm) {
  pwm_left = pwm.data;
}

void pwm_right_cb(const std_msgs::Int16& pwm) {
  pwm_right = pwm.data;
}

ros::Subscriber<std_msgs::Int16> pwm_left_subscriber("pwm_left", &pwm_left_cb);
ros::Subscriber<std_msgs::Int16> pwm_right_subscriber("pwm_right", &pwm_right_cb);

int main(void) {

    // initialize tiva-c @ 80mhz
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    init_system();
    init_motors(false, false);

    nh.initNode();

    nh.subscribe(pwm_left_subscriber);
    nh.subscribe(pwm_right_subscriber);

    while(1) {

      current_time = nh.now();

      // do processing here.

      if(pwm_left >= 0) { left_phase(FORWARD); } else if(pwm_left < 0) { left_phase(REVERSE); }
      left_pwm(abs(pwm_left));

      if(pwm_right >= 0) { right_phase(FORWARD); } else if(pwm_right < 0) { right_phase(REVERSE); }
      right_pwm(abs(pwm_right));

      nh.spinOnce();
      nh.getHardware()->delay(100);
      marked_time = current_time;

    }

}