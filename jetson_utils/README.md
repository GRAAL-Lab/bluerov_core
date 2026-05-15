# jetson_utils

Jetson-side ROS 2 utilities and launch helpers for the BlueROV stack.

## What it provides

- `switch_monitor` node:
  - Reads Jetson GPIO pin **35** (BOARD numbering)
  - Publishes safety switch state on `/auv/safety_switch` (`std_msgs/msg/Bool`)
- `main_launcher.launch.py`:
  - Starts a multi-package stack launcher (perception, mission control, KCL, logger, bridge)

## Build

From workspace root:

```bash
colcon build --packages-select jetson_utils
source install/setup.bash
```

## Usage

Run safety switch monitor:

```bash
ros2 run jetson_utils switch_monitor
```

Run integrated launcher:

```bash
ros2 launch jetson_utils main_launcher.launch.py
```

## Notes

- This package is `ament_python`.
- The launcher expects other packages to be available in the same workspace/environment.
- `switch_monitor` requires Jetson GPIO support and proper runtime permissions.
