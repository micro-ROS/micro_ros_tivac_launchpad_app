cmake_minimum_required(VERSION 2.8.3)
project(rosrider_firmware)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosserial_client
  rosserial_tivac
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac
  SCRIPT make_libraries_tiva
)

rosserial_configure_client(
  DIRECTORY ros_simple
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(ros_simple ros_simple.axf ALL)
rosserial_add_client_target(ros_simple flash)
rosserial_add_client_target(ros_simple size)

#rosserial_configure_client(
#  DIRECTORY ros_odometry
#  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
#)
#rosserial_add_client_target(ros_odometry ros_odometry.axf ALL)
#rosserial_add_client_target(ros_odometry flash)
#rosserial_add_client_target(ros_odometry size)

rosserial_configure_client(
  DIRECTORY ros_pid
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(ros_pid ros_pid.axf ALL)
rosserial_add_client_target(ros_pid flash)
rosserial_add_client_target(ros_pid size)

rosserial_configure_client(
  DIRECTORY ros_monitor
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(ros_monitor ros_monitor.axf ALL)
rosserial_add_client_target(ros_monitor flash)
rosserial_add_client_target(ros_monitor size)

rosserial_configure_client(
  DIRECTORY ros_encoders
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(ros_encoders ros_encoders.axf ALL)
rosserial_add_client_target(ros_encoders flash)
rosserial_add_client_target(ros_encoders size)

rosserial_configure_client(
  DIRECTORY ros_motors
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(ros_motors ros_motors.axf ALL)
rosserial_add_client_target(ros_motors flash)
rosserial_add_client_target(ros_motors size)

rosserial_configure_client(
  DIRECTORY ros_leds
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(ros_leds ros_leds.axf ALL)
rosserial_add_client_target(ros_leds flash)
rosserial_add_client_target(ros_leds size)
