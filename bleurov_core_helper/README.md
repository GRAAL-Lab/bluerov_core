# AUV Core Helper Package

## Overview

The **AUV Core Helper** package provides utility functions and tools to support **AUV_CORE_ROS2** in its operation. It includes functionalities for parameter loading, publishing vehicle state messages, and converting angular velocities to Euler rates. This package is designed to be a lightweight.


## Installation

### Prerequisites
To build and use the `AUV Core Helper` package, ensure the following dependencies are installed:

- [Eigen](https://eigen.tuxfamily.org/) (for linear algebra computations)
- [PkgConfig](https://github.com/pkgconf/pkgconf) (for configuration management)
- [ROS 2](https://docs.ros.org/) (tested with Humble)

### Building the package
   ```bash
   git clone https://bitbucket.org/isme_robotics/auv_core_ros2/src/main/
   colcon build --packages-select bleurov_core_helper
   source install/setup.bash
   ```


## Topics

The package defines the following message types:

- `PoseStamped`: Used for pose-related information.
- `ControlCommand`: Custom service for control commands.

The shared topic-name header defines `/auv/velocity_desired` as
`bleurov_core_helper::topicnames::velocity_desired`. KCL publishes
`geometry_msgs/msg/TwistStamped` on this topic in the `auv_base_link` FLU frame.

## KCL to ArduSub velocity mapping

`param/ctrl/velocity_rc_override.conf` is the single configuration source for
the bridge's KCL velocity to ArduSub `RC_CHANNELS_OVERRIDE` mapping. It contains
the KCL velocity limits, ArduSub channel map, PWM min/neutral/max values,
command timeout, frame ID, and output rate.

The default output rate is 100 Hz, matching KCL's `dt = 0.01 s`. This is
appropriate for the local Ethernet/UDP BlueROV connection and prevents the
bridge from adding a slower 20 Hz sample-and-hold stage. The rate remains
configurable through `rate_hz`.

# System Architecture
See the [Architecture Diagram](sw.md).


## Author Details

- [RAMI 2025 Team](https://www.graal.dibris.unige.it/studentcompetition)
#### Team Leader
- **LinkedIn**: [Youssef Attia](https://www.linkedin.com/)
- **Email**: youssef.attia@edu.unige.it

## License

This project is licensed under the MIT License. See the `LICENSE.md` file for details.
