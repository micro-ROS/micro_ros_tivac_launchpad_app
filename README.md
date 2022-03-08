<br/>

<a>
   <p align="center">
      <img width="40%" src=".images/ti.png">
      <img style="padding-left:10vw" width="40%" src=".images/microros_logo.png">
   </p>
</a>
<br/>

# micro-ROS app for TI Tiva™ C Series TM4C123G

<!-- [![CI](https://github.com/micro-ROS/micro_ros_azure_rtos_app/actions/workflows/ci.yml/badge.svg)](https://github.com/micro-ROS/micro_ros_azure_rtos_app/actions/workflows/ci.yml) -->

This example application has been tested in TI Tiva™ C Series TM4C123GXL LaunchPad using TivaWare SDK  2.2.0.295

## Dependencies

This component needs `colcon` and other Python 3 packages in order to build micro-ROS packages:

```bash
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
```

## Usage

1. Clone recursively this repo:

```bash
git clone --recursive https://github.com/micro-ROS/micro_ros_tivac_launchpad_app.git
cd micro_ros_tivac_launchpad_app
```

2. Download TivaWare™ for C Series software development kit (SDK) [here](https://www.ti.com/tool/SW-TM4C)
3. Unzip the SDK to the root of this micro-ROS project. *Linux users: just open and unzip the .exe file*
4. Make sure that the SDK is in folder named `tivaware_c_series` or change its location in the `Makefile`

5. Build the project

```bash
make -j$(nproc)
```

4. Flash the board using [lm4tools](https://github.com/utzig/lm4tools)

```bash
lm4flash gcc/microros_tivac.bin
```

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.
