# udp_camera

ROS 2 package that receives an H.264 UDP stream and republishes frames as ROS images.

## What it provides

- Node: `udp_camera_node` (script: `camera_node.py`)
- Publishes `sensor_msgs/msg/Image` on:
  - `camera/image_raw`
- Current stream pipeline uses GStreamer with a Jetson hardware decoder (`nvv4l2decoder`).

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

## Notes

- The UDP pipeline is currently configured in code for port `5600` and H.264 RTP input.
- Runtime dependencies include OpenCV, `cv_bridge`, and GStreamer support.
