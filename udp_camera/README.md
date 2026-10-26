# udp_camera

ROS 2 package that receives an H.264 UDP stream and republishes frames as ROS images.

## What it provides

- Node: `udp_camera_node` (script: `camera_node.py`)
- Publishes `sensor_msgs/msg/Image` on:
  - `camera/image_raw`
- The stream uses GStreamer with the CPU H.264 decoder (`avdec_h264`).

## Build

From workspace root:

```bash
colcon build --packages-select udp_camera
source install/setup.bash
```

## Usage

Run the node:

```bash
ros2 run udp_camera camera_node.py
```

The default UDP port is `5601`. Override it with:

```bash
ros2 run udp_camera camera_node.py --ros-args -p udp_port:=5600
```

## Notes

- The UDP pipeline expects H.264 RTP input.
- Runtime dependencies include OpenCV, `cv_bridge`, and GStreamer support.
