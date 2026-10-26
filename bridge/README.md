# bridge

BlueROV2 MAVLink bridge with optional Jetson-optimized camera ingest.

## Camera stream

Camera support is excluded from the default build, so the bridge does not create or
publish `/auv/camera/image_raw` or `/auv/camera/camera_info` during normal
startup and does not require GStreamer development packages. The bridge can
optionally publish the BlueROV camera stream from
`udp://192.168.2.3:5600` as
`sensor_msgs/msg/Image` on `/auv/camera/image_raw`.

`camera_source_uri` can also point at an `rtsp://...` source if you want to test
an RTSP stream from BlueOS instead of UDP RTP.

By default it uses the Jetson hardware H.264 decoder:

- `nvv4l2decoder`
- `nvvidconv`
- `appsink`

If the Jetson pipeline cannot start, the bridge falls back to a CPU decode path.

## Configuration

Bridge connection settings and every ROS topic/service name live in
`bridge/param/bridge.yaml` as ordinary ROS parameters.

The bridge also publishes `sensor_msgs/msg/CameraInfo` on
`/auv/camera/camera_info` so RViz camera displays can subscribe cleanly.

The same camera settings can also be overridden with the ROS parameter file at
`src/bridge/param/bridge.yaml`.

The ROS image topic is a preview stream. The bridge keeps the original H.264
transport compressed, decodes with the Jetson hardware decoder when available,
and limits the raw ROS preview by resize and maximum publish rate. The default
publish path now uses `rgba8` so the Jetson path stays on a straight row copy
instead of repacking every pixel on the CPU.

Available camera parameters:

- `camera_enabled`
- `camera_source_uri`
- `camera_rtp_caps`
- `camera_topic`
- `camera_info_topic`
- `camera_frame_id`
- `camera_use_hw_decoder`
- `camera_qos_reliable`
- `camera_enable_max_performance`
- `camera_preview_width`
- `camera_preview_height`
- `camera_preview_max_fps`
- `camera_output_encoding`
- `camera_info_publish_rate_hz`
- `camera_rtp_latency_ms`
- `camera_udp_buffer_size_bytes`
- `camera_appsink_max_buffers`
- `camera_appsink_drop`
- `camera_pipeline_override`

## Build

```bash
colcon build --packages-select bridge
```

To explicitly compile camera support, pass
`--cmake-args -DBRIDGE_ENABLE_CAMERA=ON`; it remains disabled at runtime unless
`camera_enabled` is also set to `true`.

## Launch bridge and sensors

The combined launch starts the MAVLink bridge, Sonar 3D-15 driver, A50 DVL
driver, and UDP camera driver:

```bash
ros2 launch bridge bluerov_sensors.launch.py
```

It uses the existing driver defaults: host Ethernet `192.168.2.1`, BlueOS
`192.168.2.2`, sonar `192.168.2.197`, and DVL `192.168.2.125`. The sonar is
also reachable at its fallback address `192.168.194.96`. Override the sensor addresses with
`sonar_ip:=<address>` and `dvl_ip:=<address>`.

The ROS camera listens on UDP port `5601` by default, leaving `5600` available
for QGroundControl. Override it with `camera_port:=<port>` when launching.

The UDP camera driver publishes `sensor_msgs/msg/Image` on
`/camera/image_raw`. The `bag_recored` executable records that stream together
with the sonar point cloud and the sonar driver's IMU topic,
`/sonar/imu/data_raw`.

Camera recording is enabled by default. Disable or explicitly enable it with:

```bash
ros2 run bridge bag_recored --no-camera -o bag_without_camera
ros2 run bridge bag_recored --camera -o bag_with_camera
```
