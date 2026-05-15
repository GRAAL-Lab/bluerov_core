# kcl

Kinematic Control Layer (KCL) package for BlueROV motion/state control in ROS 2.

## What it provides

- Main node: `kinematic_control_layer_node`
- Action server for mission/controller requests: `/set_kcl_state` (`auv_core_helper/action/SetKCL`)
- Internal FSM states for:
  - `IDLE`
  - `HOLD`
  - `WAYPOINT_NAVIGATION`
  - `PATH_FOLLOWING`
  - `TRAJECTORY_FOLLOWING`

## Build

From workspace root:

```bash
colcon build --packages-select auv_core_helper kcl
source install/setup.bash
```

## Usage

Run the KCL node:

```bash
ros2 run kcl kinematic_control_layer_node
```

Send action goals via CLI client script:

```bash
ros2 run kcl kcl_client.py --ros-args --params-file src/kcl/param/kcl_client.yaml
```

Run the optional PyQt GUI client:

```bash
ros2 run kcl kcl_client_gui.py
```

## Package layout

- `include/`, `src/`: C++ KCL implementation and state logic
- `scripts/`: Python action clients (`kcl_client.py`, `kcl_client_gui.py`)
- `param/`: client parameter examples
