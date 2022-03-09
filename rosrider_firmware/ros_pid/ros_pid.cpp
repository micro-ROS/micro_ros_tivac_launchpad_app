#include "rosrider.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "ros_encoders.h"
#include "ros_motors.h"
#include "ros_pid.h"

ros::NodeHandle nh;
ros::Time current_time;
ros::Time marked_time;

void setpoint_left_cb(const std_msgs::Float32& setpoint) {

  // this resets state, allows motors to slow before changing direction
  if(setpoint.data >= 0.0) {
    if(FORWARD != pid_left.get_phase()) {
        pid_left.reset_pid();
        left_pwm(0);
        pid_left.set_phase(FORWARD);
    }
  } else if(setpoint.data < 0.0) {
    if(REVERSE!=pid_left.get_phase()) {
        pid_left.reset_pid();
        left_pwm(0);
        pid_left.set_phase(REVERSE);
    }
  }

  target_rpm_left = setpoint.data;
}

void setpoint_right_cb(const std_msgs::Float32& setpoint) {

  // this resets state, allows motors to slow before chaning direction
  if(setpoint.data >= 0.0) {
    if(FORWARD!=pid_right.get_phase()) {
        pid_right.reset_pid();
        right_pwm(0);
        pid_right.set_phase(FORWARD);
    }
  } else if(setpoint.data < 0.0) {
    if(REVERSE!=pid_right.get_phase()) {
        pid_right.reset_pid();
        right_pwm(0);
        pid_right.set_phase(REVERSE);
    }
  }
  target_rpm_right = setpoint.data;
}

ros::Subscriber<std_msgs::Float32> setpoint_left_subscriber("setpoint_left", &setpoint_left_cb);
ros::Subscriber<std_msgs::Float32> setpoint_right_subscriber("setpoint_right", &setpoint_right_cb);

std_msgs::Float32 msg_rpm_left;
std_msgs::Float32 msg_rpm_right;

std_msgs::Int16 msg_pwm_left;
std_msgs::Int16 msg_pwm_right;

ros::Publisher pub_rpm_left("rpm_left", &msg_rpm_left);
ros::Publisher pub_rpm_right("rpm_right", &msg_rpm_right);

ros::Publisher pub_pwm_left("pwm_left", &msg_pwm_left);
ros::Publisher pub_pwm_right("pwm_right", &msg_pwm_right);

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

    nh.subscribe(setpoint_left_subscriber);
    nh.subscribe(setpoint_right_subscriber);

    nh.advertise(pub_rpm_left);
    nh.advertise(pub_rpm_right);
    nh.advertise(pub_pwm_left);
    nh.advertise(pub_pwm_right);

    while(1) {

      current_time = nh.now();

      double dt = current_time.toSec() - marked_time.toSec();

      update_pid(dt);

      msg_rpm_left.data = current_rpm_left;
      msg_rpm_right.data = current_rpm_right;

      msg_pwm_left.data = pwm_left;
      msg_pwm_right.data = pwm_right;

      pub_rpm_left.publish(&msg_rpm_left);
      pub_rpm_right.publish(&msg_rpm_right);
      pub_pwm_left.publish(&msg_pwm_left);
      pub_pwm_right.publish(&msg_pwm_right);

      nh.spinOnce();
      nh.getHardware()->delay(100);
      marked_time = current_time;

    }

    return(0);
}