#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from auv_core_helper.msg import PoseStamped, MissionStatus
from datetime import datetime, timezone
import os
import math
import simplekml
import signal
import sys
import time

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        # subscribers
        self.pose_sub = self.create_subscription(PoseStamped, "/auv/global/pose_actual", self.pose_callback, 10)
        self.mission_sub = self.create_subscription(MissionStatus, "/auv/mission/status", self.pose_callback, 10)

        log_dir = os.path.expanduser('~/mission_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.kml_path = os.path.join(log_dir, f"vehicle_navigation_data_{timestamp}.kml")
        self.kml = simplekml.Kml()
        
        self.last_save_time = time.time()
        self.save_interval = 10  # seconds
        
        self.latest_pose = None
        # Log pose at 1 Hz
        self.timer = self.create_timer(1.0, self.log_pose)

        self.get_logger().info(f"Logging to: {self.kml_path}")
        

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def log_pose(self):
        if self.latest_pose is None:
            return

        t = self.latest_pose.header.stamp
        timestamp = t.sec + t.nanosec * 1e-9
        dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()

        lat = self.latest_pose.position.latitude
        lon = self.latest_pose.position.longitude
        depth = self.latest_pose.depth
        yaw_deg = self.latest_pose.yaw * (180.0 / math.pi)

        p = self.kml.newpoint(name="Pose", coords=[(lon, lat)])
        p.timestamp.when = dt
        p.extendeddata.newdata(name="Heading", value=str(yaw_deg))
        p.extendeddata.newdata(name="Depth", value=str(depth))
        
        # Periodically save
        now = time.time()
        if now - self.last_save_time >= self.save_interval:
            self.save_kml()
            self.last_save_time = now

        self.get_logger().info(f"[POSE] Logged at {dt}")
        
    def save_kml(self):
        try:
            self.kml.save(self.kml_path)
            self.get_logger().info("KML saved.")
        except Exception as e:
            self.get_logger().error(f"Error saving KML: {e}")

    def destroy_node(self):
        self.kml.save(self.kml_path)
        self.get_logger().info(f"Final KML file saved: {self.kml_path}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    
    def signal_handler(sig, frame):
        node.get_logger().warn(f"Signal {sig} received. Exiting...")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Handle SIGINT and SIGTERM
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LoggerNode interrupted by user')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
        
if __name__ == '__main__':
    main()