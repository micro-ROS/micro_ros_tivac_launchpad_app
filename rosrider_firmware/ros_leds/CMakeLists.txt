cmake_minimum_required(VERSION 2.8.3)
project(ros_leds)

include_directories(${ROS_LIB_DIR})

add_definitions(-DTX_BUFFER_SIZE=1024)
add_definitions(-DRX_BUFFER_SIZE=1024)
add_definitions(-DUART_BUFFER_SIZE=1024)
add_definitions(-DROSSERIAL_BAUDRATE=115200)

generate_tivac_firmware(
  USB
  SRCS ros_leds.cpp
  INCS . ../include ${TIVA_WARE_PATH}/examples/boards/ek-tm4c123gxl/drivers
  BOARD tm4c123gxl
)
