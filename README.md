<!-- ABOUT THE PROJECT -->
## ODTC
## Operating system
Runs on 22.04, ROS2 Humble.

## Documentation
..

## Features
obstacle detection

## Dependencies
Before building the repository you will have to install the following dependencies:

* odtc (https://bitbucket.org/isme_robotics/odtc/src/main/)
* ROS2 messages (std_msgs, geographic_msgs, sensor_msgs, geometric_msgs)


## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install

The CMakeLists.txt provides also an additional BUILD_TESTS option, which by default is set to OFF. If you want to build also the tests just run:

    $ cmake .. -DBUILD_TESTS=ON

### Mantainer

* <luca.tarasi@edu.unige.it>
