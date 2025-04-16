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
   colcon build --packages-select auv_core_helper
   source install/setup.bash
   ```


## Topics

The package defines the following message types:

- `PoseStamped`: Used for pose-related information.
- `ControlCommand`: Custom service for control commands.


## Author Details

- **LinkedIn**: [Youssef Attia](https://www.linkedin.com/)
- **Email**: youssef.attia@edu.unige.it

## License

This project is licensed under the MIT License. See the `LICENSE.md` file for details.
