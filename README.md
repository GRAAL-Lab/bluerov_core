# bluerov_core

ROS 2 workspace for BlueROV core autonomy, control, perception, communications, and mission support components.

Detailed setup and node-specific usage are documented in each package folder README; this top-level file is a quick orientation guide.

## Repository structure

Top-level packages in this workspace:

| Package | Purpose | Details |
|---|---|---|
| `auv_core_helper` | Shared helpers, parameters, and common utilities | [`auv_core_helper/README.md`](./auv_core_helper/README.md) |
| `bridge` | MAVLink and camera/data bridge components | [`bridge/README.md`](./bridge/README.md) |
| `ctrl_station` | Command/control station and MQTT-related interfaces | [`ctrl_station/README.md`](./ctrl_station/README.md) |
| `logger` | Mission/navigation logging utilities | [`logger/README.md`](./logger/README.md) |
| `mission_ctrl` | Mission controller state-machine logic | [`mission_ctrl/README.md`](./mission_ctrl/README.md) |
| `perception` | Perception and obstacle tracking pipeline | [`perception/README.md`](./perception/README.md) |
| `kcl` | Vehicle control/action layer package | Package folder |
| `jetson_utils` | Jetson-side utility nodes/modules | Package folder |
| `udp_camera` | UDP camera package/integration | Package folder |

## Quick usage

### 1) Prerequisites

- Ubuntu 22.04
- ROS 2 Humble environment sourced
- `colcon` toolchain installed

### 2) Build workspace

From the repository root:

```bash
colcon build
source install/setup.bash
```

### 3) Run packages

Use package-specific commands from each folder README. Typical commands follow standard ROS 2 patterns:

```bash
ros2 run <package_name> <executable>
ros2 launch <package_name> <launch_file>
```

## Notes

- Prefer package-level READMEs for dependency lists, parameters, topics, launch files, and workflow details.
- Some directories may also contain additional scripts, configs, or support assets used by the ROS 2 packages.
