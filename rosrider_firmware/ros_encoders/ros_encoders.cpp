#include "rosrider.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "ros_encoders.h"
#include "ros_motors.h"

ros::NodeHandle nh;
ros::Time current_time;
ros::Time marked_time;

std_msgs::Int32 msg_delta_left;
std_msgs::Int32 msg_delta_right;
std_msgs::Float32 msg_rpm_left;
std_msgs::Float32 msg_rpm_right;

void pwm_left_cb(const std_msgs::Int16& pwm) {
  pwm_left = pwm.data;
}

void pwm_right_cb(const std_msgs::Int16& pwm) {
  pwm_right = pwm.data;
}

ros::Subscriber<std_msgs::Int16> pwm_left_subscriber("pwm_left", &pwm_left_cb);
ros::Subscriber<std_msgs::Int16> pwm_right_subscriber("pwm_right", &pwm_right_cb);

ros::Publisher pub_rpm_left("rpm_left", &msg_rpm_left);
ros::Publisher pub_rpm_right("rpm_right", &msg_rpm_right);
ros::Publisher pub_delta_left("delta_left", &msg_delta_left);
ros::Publisher pub_delta_right("delta_right", &msg_delta_right);

int main(void) {

    // initialize tiva-c @ 80mhz
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    init_system();
    init_motors(false, false);
    // at this instance, encoders are set at 10hz
    init_encoders(ui32SysClkFreq / 10, false, false, true, true);
    nh.initNode();

    nh.subscribe(pwm_left_subscriber);
    nh.subscribe(pwm_right_subscriber);

    nh.advertise(pub_rpm_left);
    nh.advertise(pub_rpm_right);

    nh.advertise(pub_delta_left);
    nh.advertise(pub_delta_right);

    left_pwm(0);
    right_pwm(0);

    while(1) {

      current_time = nh.now();

      double dt = current_time.toSec() - marked_time.toSec();

      // do processing here.

      if(pwm_left >= 0) { left_phase(FORWARD); } else if(pwm_left < 0) { left_phase(REVERSE); }
      left_pwm(abs(pwm_left));

      if(pwm_right >= 0) { right_phase(FORWARD); } else if(pwm_right < 0) { right_phase(REVERSE); }
      right_pwm(abs(pwm_right));

      measure_encoders();

      // notice to get signed velocity, we must multiply vel by encdir
      int32_t c_left = vel_left * encdir_left;
      int32_t c_right = vel_right * encdir_right;

      msg_rpm_left.data = c_left * ROUNDS_PER_MINUTE;
      msg_rpm_right.data = c_right * ROUNDS_PER_MINUTE;

      msg_delta_left.data = delta_left;
      msg_delta_right.data = delta_right;

      pub_rpm_left.publish(&msg_rpm_left);
      pub_rpm_right.publish(&msg_rpm_right);

      pub_delta_left.publish(&msg_delta_left);
      pub_delta_right.publish(&msg_delta_right);

      nh.spinOnce();
      nh.getHardware()->delay(100);
      marked_time = current_time;

    }

    return(0);
}

