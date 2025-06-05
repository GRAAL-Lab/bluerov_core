from rclpy.node import Node
from auv_core_helper.msg import PoseStamped, MissionStatus
from datetime import datetime, timezone
import os
import math
import simplekml
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

        self.kml_path_nav = os.path.join(log_dir, f"vehicle_navigation_data_{timestamp}.kml")
        self.kml_path_mission = os.path.join(log_dir, f"mission_status_data{timestamp}.kml")
        
        self.kml_nav = simplekml.Kml()
        self.kml_mission = simplekml.Kml()
        
        self.last_save_time = time.time()
        self.save_interval = 10  # seconds
        
        self.latest_pose = None
        self.latest_mission_status = None
        # Log pose at 1 Hz
        self.timer = self.create_timer(1.0, self.log_pose)

        self.get_logger().info(f"Logging navigation data to: {self.kml_path_nav}")
        self.get_logger().info(f"Logging mission status to: {self.kml_path_mission}")
        

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        
    def mission_callback(self, msg: MissionStatus):
        try:
            if self.is_mission_status_changed(msg):
                self.log_mission_status(msg)
        except Exception as e:
            self.get_logger().error(f"[MISSION] Logging error: {e}")

    def is_mission_status_changed(self, msg: MissionStatus) -> bool:
        subtask = getattr(msg, 'subtask', None)
        key_decision = getattr(msg, 'key_decision', None)
        event_message = getattr(msg, 'event_message', None)

        changed = (
            subtask != self.last_subtask or
            key_decision != self.last_key_decision or
            event_message != self.last_event_message
        )
        return changed

    def log_mission_status(self, msg: MissionStatus):
        subtask = getattr(msg, 'subtask', None)
        key_decision = getattr(msg, 'key_decision', None)
        event_message = getattr(msg, 'event_message', None)

        t = msg.header.stamp
        timestamp = t.sec + t.nanosec * 1e-9
        dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()

        name = f"Subtask: {subtask if subtask else 'Unknown'}"

        p = self.kml_mission.newpoint(name=name, coords=[(0, 0)])  # coords optional here

        p.timestamp.when = dt
        p.extendeddata.newdata(name="Key Decision", value=str(key_decision))
        p.extendeddata.newdata(name="Event Message", value=str(event_message))

        self.get_logger().info(f"[MISSION] Logged new status at {dt}")

        self.last_subtask = subtask
        self.last_key_decision = key_decision
        self.last_event_message = event_message

    def log_pose(self):
        if self.latest_pose is None:
            return
        try:
            t = self.latest_pose.header.stamp
            timestamp = t.sec + t.nanosec * 1e-9
            dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()

            lat = self.latest_pose.position.latitude
            lon = self.latest_pose.position.longitude
            depth = self.latest_pose.depth
            yaw_deg = self.latest_pose.yaw * (180.0 / math.pi)

            p = self.kml_nav.newpoint(name="Pose", coords=[(lon, lat)])
            p.timestamp.when = dt
            p.extendeddata.newdata(name="Heading", value=str(yaw_deg))
            p.extendeddata.newdata(name="Depth", value=str(depth))

            self.get_logger().info(f"[POSE] Logged at {dt}")
            
            # Periodic save
            current_time = time.time()
            if current_time - self.last_save_time > self.save_interval:
                self.save_logs()
                self.last_save_time = current_time
        
        except Exception as e:
            self.get_logger().error(f"[POSE] Logging error: {e}")
        
    def save_logs(self):
        try:
            self.kml_nav.save(self.kml_path_nav)
            self.kml_mission.save(self.kml_path_mission)
            self.get_logger().info("KMLs saved.")
        except Exception as e:
            self.get_logger().error(f"Error saving KMLs: {e}")

    def destroy_node(self):
        self.save_logs()
        self.get_logger().info(f"Final KMLs file saved: {self.kml_path_nav}")
        super().destroy_node()