# bridge

BlueROV2 MAVLink bridge with Jetson-optimized camera ingest.

## Camera stream

The bridge can publish the BlueROV camera stream from `udp://192.168.2.3:5600` as
`sensor_msgs/msg/Image` on `/auv/camera/image_raw`.

`camera_source_uri` can also point at an `rtsp://...` source if you want to test
an RTSP stream from BlueOS instead of UDP RTP.

By default it uses the Jetson hardware H.264 decoder:

- `nvv4l2decoder`
- `nvvidconv`
- `appsink`

If the Jetson pipeline cannot start, the bridge falls back to a CPU decode path.

## Configuration

System defaults live in
`src/auv_core_helper/param/ctrl/bridge.conf`.

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
colcon build --packages-select auv_core_helper bridge
```
