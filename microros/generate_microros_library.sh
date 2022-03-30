#!/bin/bash

# Check if colcon is available
if ! command -v colcon &> /dev/null
then
    echo "colcon could not be found"
    exit 1
fi

BUILD_TYPE=Debug

COMPONENT_DIR=$(pwd)
INSTALL_DIR=$(pwd)/install

X_PREFIX=$1
X_CC=$X_PREFIX-gcc
X_CXX=$X_PREFIX-gcc
X_AR=$X_PREFIX-ar

echo "Using X_PREFIX: $X_PREFIX"

CFLAGS_INTERNAL="-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -MD -std=c99 -Wall -pedantic -DPART_TM4C123GH6PM -c -Os -DTARGET_IS_TM4C123_RB1 -DUART_BUFFERED --specs=nosys.specs -Dgcc -DCLOCK_MONOTONIC=0"
CXXFLAGS_INTERNAL="-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -MD -Wall -pedantic -DPART_TM4C123GH6PM -c -Os -DTARGET_IS_TM4C123_RB1 -DUART_BUFFERED --specs=nosys.specs -Dgcc -DCLOCK_MONOTONIC=0"

rm -rf $INSTALL_DIR/install
rm -rf $INSTALL_DIR/toolchain.cmake
rm -rf $INSTALL_DIR/libmicroros.a
rm -rf $INSTALL_DIR/compiler_version.txt
rm -rf $INSTALL_DIR/micro_ros_src/build
rm -rf $INSTALL_DIR/micro_ros_src/install
rm -rf $INSTALL_DIR/micro_ros_src/log

mkdir -p $INSTALL_DIR/micro_ros_dev;
pushd $INSTALL_DIR/micro_ros_dev > /dev/null
	git clone -b master https://github.com/ament/ament_cmake src/ament_cmake;
	git clone -b master https://github.com/ament/ament_lint src/ament_lint;
	git clone -b master https://github.com/ament/ament_package src/ament_package;
	git clone -b ros2 https://github.com/ament/googletest src/googletest;
	git clone -b master https://github.com/ros2/ament_cmake_ros src/ament_cmake_ros;
	git clone -b master https://github.com/ament/ament_index src/ament_index;
    colcon build --cmake-args -DBUILD_TESTING=OFF;
popd > /dev/null

mkdir -p $INSTALL_DIR/micro_ros_src;
pushd $INSTALL_DIR/micro_ros_src > /dev/null
	git clone -b ros2 https://github.com/eProsima/micro-CDR src/micro-CDR;
	git clone -b ros2 https://github.com/eProsima/Micro-XRCE-DDS-Client src/Micro-XRCE-DDS-Client;
	git clone -b master https://github.com/micro-ROS/rcl src/rcl;
	git clone -b master https://github.com/ros2/rclc src/rclc;
	git clone -b master https://github.com/micro-ROS/rcutils src/rcutils;
	git clone -b main https://github.com/micro-ROS/micro_ros_msgs src/micro_ros_msgs;
	git clone -b main https://github.com/micro-ROS/rmw-microxrcedds src/rmw-microxrcedds;
	git clone -b master https://github.com/micro-ROS/rosidl_typesupport src/rosidl_typesupport;
	git clone -b main https://github.com/micro-ROS/rosidl_typesupport_microxrcedds src/rosidl_typesupport_microxrcedds;
	git clone -b master https://github.com/ros2/rosidl src/rosidl;
	git clone -b master https://github.com/ros2/rmw src/rmw;
	git clone -b master https://github.com/ros2/rcl_interfaces src/rcl_interfaces;
	git clone -b master https://github.com/ros2/rosidl_defaults src/rosidl_defaults;
	git clone -b master https://github.com/ros2/unique_identifier_msgs src/unique_identifier_msgs;
	git clone -b master https://github.com/ros2/common_interfaces src/common_interfaces;
	git clone -b master https://github.com/ros2/test_interface_files src/test_interface_files;
	git clone -b master https://github.com/ros2/rmw_implementation src/rmw_implementation;
	git clone -b master https://github.com/ros2/rcl_logging src/rcl_logging;
	git clone -b master https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing src/ros2_tracing;
	# git clone -b main https://github.com/micro-ROS/micro_ros_utilities src/micro_ros_utilities;

    touch src/rosidl/rosidl_typesupport_introspection_cpp/COLCON_IGNORE;
    touch src/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE;
    touch src/rcl_logging/rcl_logging_spdlog/COLCON_IGNORE;
    touch src/rclc/rclc_examples/COLCON_IGNORE;
	touch src/rcl/rcl_yaml_param_parser/COLCON_IGNORE;


    git clone -b ros2 https://github.com/ros2/geometry2
    cp -R geometry2/tf2_msgs src/tf2_msgs
    rm -rf geometry2

    # cp -R $COMPONENT_DIR/extra_packages/* src

    unset AMENT_PREFIX_PATH;
    unset RMW_IMPLEMENTATION;

    . $INSTALL_DIR/micro_ros_dev/install/local_setup.sh;

    ESCAPED_CFLAGS_INTERNAL=$(echo $CFLAGS_INTERNAL | sed 's/\//\\\//g' | sed 's/"//g')
    ESCAPED_CXXFLAGS_INTERNAL=$(echo $CXXFLAGS_INTERNAL | sed 's/\//\\\//g' | sed 's/"//g')
    ESCAPED_X_CC=$(echo $X_CC | sed 's/\//\\\//g' | sed 's/"//g')
    ESCAPED_X_CXX=$(echo $X_CXX | sed 's/\//\\\//g' | sed 's/"//g')

    cat $COMPONENT_DIR/toolchain.cmake.in | \
        sed "s/@CMAKE_C_COMPILER@/$ESCAPED_X_CC/g" | \
        sed "s/@CMAKE_CXX_COMPILER@/$ESCAPED_X_CXX/g" | \
        sed "s/@CFLAGS@/$ESCAPED_CFLAGS_INTERNAL/g" | \
        sed "s/@CXXFLAGS@/$ESCAPED_CXXFLAGS_INTERNAL/g" \
        > $INSTALL_DIR/toolchain.cmake

    colcon build \
        --merge-install \
        --packages-ignore-regex=.*_cpp \
        --metas $COMPONENT_DIR/colcon.meta \
        --cmake-force-configure \
        --cmake-clean-cache \
        --cmake-args \
        "--no-warn-unused-cli" \
        --log-level=ERROR \
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
        -DTHIRDPARTY=ON \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_TESTING=OFF \
        -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
        -DCMAKE_TOOLCHAIN_FILE=$INSTALL_DIR/toolchain.cmake \
        -DCMAKE_VERBOSE_MAKEFILE=ON;

popd > /dev/null

# Create packaged library
mkdir -p $INSTALL_DIR/libmicroros; cd $INSTALL_DIR/libmicroros; \
for file in $(find $INSTALL_DIR/micro_ros_src/install/lib/ -name '*.a'); do \
    folder=$(echo $file | sed -E "s/(.+)\/(.+).a/\2/"); \
    mkdir -p $folder; cd $folder; ${X_AR} x $file; \
    for f in *; do \
        mv $f ../$folder-$f; \
    done; \
    cd ..; rm -rf $folder; \
done ; \
${X_AR} rc libmicroros.a $(ls *.o *.obj 2> /dev/null); mkdir -p $INSTALL_DIR; cp libmicroros.a $INSTALL_DIR; ranlib $INSTALL_DIR/libmicroros.a; \
cp -R $INSTALL_DIR/micro_ros_src/install/include $INSTALL_DIR/; \
cd ..; rm -rf libmicroros;

######## Fix include paths  ########
pushd $INSTALL_DIR/micro_ros_src > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null

apt -y install rsync
for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r $INSTALL_DIR/include/${var}/${var}/* $INSTALL_DIR/include/${var}/
    rm -rf $INSTALL_DIR/include/${var}/${var}/
done

# Print compiler info
echo "C Compiler" > $INSTALL_DIR/compiler_version.txt;
echo "------------------------" >> $INSTALL_DIR/compiler_version.txt;
${X_CC} -v &>> $INSTALL_DIR/compiler_version.txt
echo "" >> $INSTALL_DIR/compiler_version.txt;
echo "C++ Compiler" >> $INSTALL_DIR/compiler_version.txt;
echo "------------------------" >> $INSTALL_DIR/compiler_version.txt;
${X_CXX} -v &>> $INSTALL_DIR/compiler_version.txt