#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UdpCameraNode(Node):
    def __init__(self):
        super().__init__('udp_camera_node')
        self.declare_parameter('udp_port', 5601)
        udp_port = self.get_parameter('udp_port').value

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        gst_str = (
            f"udpsrc port={udp_port} buffer-size=2097152 "
            "caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink max-buffers=1 drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        self.get_logger().info(
            f"Listening for the H.264 RTP camera stream on UDP port {udp_port}"
        )

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video stream")
            raise RuntimeError("Could not open video stream")
            

        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to get frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UdpCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
