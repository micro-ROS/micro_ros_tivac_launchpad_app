cmake_minimum_required(VERSION 2.8.3)
project(ros_simple)

include_directories(${ROS_LIB_DIR})

add_definitions(-DTX_BUFFER_SIZE=1024)
add_definitions(-DRX_BUFFER_SIZE=1024)
add_definitions(-DUART_BUFFER_SIZE=1024)
add_definitions(-DROSSERIAL_BAUDRATE=230400)

generate_tivac_firmware(
  USB
  SRCS ros_simple.cpp ../ros_encoders/TivaQEI.cpp ../include/ina219.c ../include/ina219_tiva.c
  INCS . ../include ../ros_encoders ../ros_motors ../ros_leds ../ros_monitor ${TIVA_WARE_PATH}/examples/boards/ek-tm4c123gxl/drivers
  LIBS ${TIVA_WARE_PATH}/driverlib/gcc/libdriver.a ${TIVA_WARE_PATH}/sensorlib/gcc/libsensor.a
  BOARD tm4c123gxl
)
