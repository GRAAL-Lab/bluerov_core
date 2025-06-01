# Control Toolbox
This library is a collection of control tools such as PID and adaptive controllers and signal filtering. The namespace enclosing all the library facilites is \p ctb (Control Tool Box).

## Components
The list of components is the following:

1. PID Controller: **ctb::DigitalPID**
2. Virtual Frame: **ctb::VirtualFrame**
3. Kalman Filter: **ctb::ExtendedKalmanFilter**

## Dependencies

* **rml**: Robotic Mathematical Library (https://bitbucket.org/isme_robotics/rml/src)
* **Geographic Library**: `sudo apt-get install -y libgeographic-dev`)
* **libconfig++**: `sudo apt install libconfig++-dev`
