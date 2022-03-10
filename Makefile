ROOT=tivaware_c_series

DEBUG=1
VERBOSE=1
COMPILER=gcc

PART=TM4C123GH6PM

include ${ROOT}/makedefs

${COMPILER}${SUFFIX}/%.o: %.cpp
	echo ${CC} ${CFLAGS} -D${COMPILER} -o ${@} ${<}
	@${CC} ${CFLAGS} -D${COMPILER} -o ${@} ${<}

VPATH=${ROOT}/examples/boards/ek-tm4c123gxl/drivers
VPATH+=${ROOT}/utils
VPATH+=src
VPATH+=rosrider_firmware/ros_encoders
VPATH+=rosrider_firmware/include

IPATH=${ROOT}/examples/boards/ek-tm4c123gxl/
IPATH+=${ROOT}
IPATH+=microros/install/include
IPATH+=src
IPATH+=./
IPATH+=rosrider_firmware/include
IPATH+=rosrider_firmware/include


all: ${COMPILER}
all: ${COMPILER}/microros_tivac.bin

clean:
	@rm -rf ${COMPILER} ${wildcard *~}

${COMPILER}:
	@mkdir -p ${COMPILER}

OBJS=\
${COMPILER}/microros.o \
${COMPILER}/microros_init.o \
${COMPILER}/TivaQEI.o \
${COMPILER}/ina219.o \
${COMPILER}/ina219_tiva.o \
${COMPILER}/syscalls.o \
${COMPILER}/microros_usbcdc_transport.o \
${COMPILER}/microros_time.o \
${COMPILER}/microros_allocators.o \
${COMPILER}/microros_parameters.o \
${COMPILER}/startup_gcc.o \
${COMPILER}/uartstdio.o \
${COMPILER}/main.o \
${COMPILER}/usb_serial_structs.o \
${COMPILER}/ustdlib.o \
${ROOT}/usblib/gcc/libusb.a \
${ROOT}/driverlib/gcc/libdriver.a \
microros/install/libmicroros.a

${COMPILER}/microros_tivac.elf: ${OBJS}
${COMPILER}/microros_tivac.elf: microros_tivac.ld

${COMPILER}/microros_usbcdc_transport.o: microros/install/libmicroros.a
${COMPILER}/microros.o: microros/install/libmicroros.a
${COMPILER}/microros_parameters.o: microros/install/libmicroros.a

# SCATTERgcc_microros_tivac=microros_tivac.ld
# ENTRY_microros_tivac=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C123_RB1 -DUART_BUFFERED

${COMPILER}/microros_tivac.elf:
	${PREFIX}-gcc -o ${COMPILER}/microros_tivac.elf ${CPU} ${FPU} -Wl,--gc-sections -Wl,--print-memory-usage -T "microros_tivac.ld" --specs=nano.specs ${OBJS}

${COMPILER}/microros_tivac.bin: ${COMPILER}/microros_tivac.elf
	${PREFIX}-objcopy -O binary ${COMPILER}/microros_tivac.elf ${COMPILER}/microros_tivac.bin

microros/install/libmicroros.a:
	@echo "Building libmicroros.a"
	cd microros && ./generate_microros_library.sh ${PREFIX} && cd ../

flash: ${COMPILER}/microros_tivac.bin
	lm4flash ${COMPILER}/microros_tivac.bin

size:
	${PREFIX}-nm --print-size --size-sort --radix=d ${COMPILER}/microros_tivac.elf | grep -E " b | B | d | D "