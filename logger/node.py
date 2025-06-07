from rclpy.node import Node
from auv_core_helper.msg import PoseStamped, MissionStatus
from datetime import datetime, timezone
from logger.utilities import STATE_NAME_MAP
import os
import math
import simplekml
import time

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        
         # Get team name and create RAMI-compliant folder structure
        self.team_name = self.declare_parameter('team_name', 'UniGe_ISME').get_parameter_value().string_value
        self.mission_start_time = datetime.now(tz=timezone.utc)
        
        # Create RAMI-compliant folder name: TEAM_X_YYYYMMDD_HHMM
        folder_timestamp = self.mission_start_time.strftime("%Y%m%d_%H%M")
        folder_name = f"{self.team_name}_{folder_timestamp}"
        
        # Create log directory with RAMI-compliant structure
        log_dir = os.path.expanduser('~/mission_logs')
        self.mission_dir = os.path.join(log_dir, folder_name)
        os.makedirs(self.mission_dir, exist_ok=True)
        file_timestamp = self.mission_start_time.strftime("%Y%m%d_%H%M%S")
       
        # subscribers
        self.pose_sub = self.create_subscription(PoseStamped, "/auv/global/pose_actual", self.pose_callback, 10)
        self.mission_sub = self.create_subscription(MissionStatus, "/auv/mission/status", self.mission_callback, 10)

        self.kml_path_nav = os.path.join(self.mission_dir, f"vehicle_navigation_data_{file_timestamp}.kml")
        self.kml_path_mission = os.path.join(self.mission_dir, f"mission_status_data_{file_timestamp}.kml")
        
        self.kml_nav = simplekml.Kml()
        self.kml_mission = simplekml.Kml()
        
        self.last_save_time = time.time()
        self.save_interval = 10  # seconds
        
        self.latest_pose = None
        self.latest_mission_status = None
        
        # Initialize mission status tracking variables
        self.last_state = None
        self.last_state_object = None
        
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
        state = getattr(msg, 'state', None)
        state_object = getattr(msg, 'state_object', None)

        changed = (
            state != self.last_state or
            state_object != self.last_state_object
        )
        return changed

    def log_mission_status(self, msg: MissionStatus):
        state = getattr(msg, 'state', None)
        state_object = getattr(msg, 'state_object', None)

        t = msg.stamp
        timestamp = t.sec + t.nanosec * 1e-9
        dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()

        readable_state = STATE_NAME_MAP.get(state, state if state else "Unknown")
        name = f"Subtask: {readable_state}"

        p = self.kml_mission.newpoint(name=name, coords=[(0, 0)])  # coords optional here

        p.timestamp.when = dt
        p.extendeddata.newdata(name="Key Decision and Event Message", value=str(state_object))

        self.get_logger().info(f"[MISSION] Logged new status '{readable_state}' at {dt}")

        self.last_state = state
        self.last_state_object = state_object

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