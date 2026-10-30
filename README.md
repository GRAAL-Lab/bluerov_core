# bluerov_core

ROS 2 workspace for BlueROV core autonomy, control, perception, communications, and mission support components.

Detailed setup and node-specific usage are documented in each package folder README; this top-level file is a quick orientation guide.

## Repository structure

Top-level packages in this workspace:

| Package | Purpose | Details |
|---|---|---|
| `bleurov_core_helper` | Shared helpers, parameters, and common utilities | [`bleurov_core_helper/README.md`](./bleurov_core_helper/README.md) |
| `bridge` | MAVLink and camera/data bridge components | [`bridge/README.md`](./bridge/README.md) |
| `bluerov_video_viewer` | Low-latency BlueROV UDP video bridge and viewer | [`bluerov_video_viewer/README.md`](./bluerov_video_viewer/README.md) |

## Quick usage

### 1) Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy environment sourced
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
