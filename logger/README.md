# README #
# ROS 2 Logger Node (Mission KML Logger)

This ROS 2 package provides a logging node that records vehicle navigation data and mission status messages into KML files for post-mission analysis and visualization 

## Usage

### 1. Build the package

```bash
colcon build --packages-select logger
source install/setup.bash
```
### 2. Run the logger node
```
ros2 run logger logger_node
```

## Features

- Subscribes to:
  - `/auv/global/pose_actual` (`PoseStamped`): vehicle geolocation and orientation
  - `/auv/mission/status` (`MissionStatus`): mission progress updates
- Logs data into:
  - `vehicle_navigation_data_<timestamp>.kml`
  - `mission_status_data_<timestamp>.kml`
- RAMI-compliant folder structure:
  - `~/mission_logs/UniGe_ISME_YYYYMMDD_HHMM/`
- Periodic log saving every 10 seconds

## Output Structure

All logs are saved to a mission folder using this format:

```text
~/mission_logs/UniGe_ISME_20250605_1126/
├── vehicle_navigation_data_20250605_112647.kml
└── mission_status_data_20250605_112647.kml
```

