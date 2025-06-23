from rclpy.node import Node
from auv_core_helper.msg import PoseStamped, MissionStatus
from image_pipeline_msgs.msg import Obstacles
from sensor_msgs.msg import Image
from datetime import datetime, timezone
from utils.utilities import STATE_NAME_MAP, TOPICS_NAMES
import os
import math
import simplekml
import time
import cv2
from cv_bridge import CvBridge

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
        self.pose_sub = self.create_subscription(PoseStamped, TOPICS_NAMES["Pose"], self.pose_callback, 10)
        self.mission_sub = self.create_subscription(MissionStatus, TOPICS_NAMES["MissionStatus"], self.mission_callback, 10)
        self.perception_sub = self.create_subscription(Obstacles, TOPICS_NAMES["Obstacles"],self.perception_callback,10)
        self.image_sub = self.create_subscription(Image, TOPICS_NAMES["Images"], self.image_callback, 10)

        self.kml_path_nav = os.path.join(self.mission_dir, f"vehicle_navigation_data_{file_timestamp}.kml")
        self.kml_path_mission = os.path.join(self.mission_dir, f"mission_status_data_{file_timestamp}.kml")
        self.kml_path_objects = os.path.join(self.mission_dir, f"object_recognition_data_{file_timestamp}.kml")
        self.image_save_path = os.path.join(self.mission_dir, "object_images")
        os.makedirs(self.image_save_path, exist_ok=True)
        
        self.kml_nav = simplekml.Kml()
        self.kml_mission = simplekml.Kml()
        self.kml_objects = simplekml.Kml()
        
        self.last_save_time = time.time()
        self.save_interval = 10  # seconds
        
        self.latest_pose = None
        self.latest_mission_status = None
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Initialize mission status tracking variables
        self.last_state = None
        self.last_state_object = None
        self.seen_buoy_ids = set()
        self.seen_pipe_ids = set()
        self.seen_marker_ids = set()
        self.seen_number_ids = set()

        # Log pose at 1 Hz
        self.timer = self.create_timer(1.0, self.log_pose)

        self.get_logger().info(f"Logging navigation data to: {self.kml_path_nav}")
        self.get_logger().info(f"Logging mission status to: {self.kml_path_mission}")
        self.get_logger().info(f"Logging object recognition data to: {self.kml_path_objects}")
        

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
    
    def image_callback(self, msg: Image):
        try:
            self.latest_image = msg  # Save latest image for perception callback  
        except Exception as e:
            self.get_logger().error(f"[IMAGE] Image callback failed: {e}")

        
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
    
    def save_image(self, timestamp):
        if self.latest_image is None:
            self.get_logger().warn("[IMAGE] No latest image available to save")
            return ""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            filename = f"object_{int(timestamp * 1000)}.jpg"
            filepath = os.path.join(self.image_save_path, filename)
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"[IMAGE] Saved image: {filename}")
            return filename
        except Exception as e:
            self.get_logger().error(f"[IMAGE] Error saving image: {e}")
            return ""


    def perception_callback(self, msg):
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()
            
            # Skip if no image available yet
            if self.latest_image is None:
                self.get_logger().warn("[IMAGE] No latest image available to save yet, skipping saving image.")
                return
            
            object_found = False

            # === BUOYS ===
            for buoy in msg.buoys:
                if buoy.id not in self.seen_buoy_ids:
                    self.seen_buoy_ids.add(buoy.id)
                    object_found = True
                    pos = buoy.pose.pose.position
                    lat = pos.latitude
                    lon = pos.longitude
                    depth = -pos.altitude

                    object_id = f"buoy_{buoy.id}"
                    filename = self.save_image(timestamp)

                    placemark = self.kml_objects.newpoint(name=f"Buoy {buoy.id}")
                    placemark.timestamp.when = dt
                    placemark.coords = [(lon, lat, depth)]
                    placemark.extendeddata.newdata(name="Target ID", value=object_id)
                    placemark.extendeddata.newdata(name="Latitude", value=str(lat))
                    placemark.extendeddata.newdata(name="Longitude", value=str(lon))
                    placemark.extendeddata.newdata(name="Depth", value=str(depth))
                    placemark.extendeddata.newdata(
                        name="Features", value=f"radius={buoy.radius}, color={buoy.color}"
                    )
                    placemark.extendeddata.newdata(name="Image", value=filename)

            # === MARKERS ===
            for marker in msg.markers:
                if marker.id not in self.seen_marker_ids:
                    self.seen_marker_ids.add(marker.id)
                    object_found = True
                    pos = marker.pose.pose.position
                    lat = pos.latitude
                    lon = pos.longitude
                    depth = -pos.altitude

                    object_id = f"marker_{marker.id}"
                    filename = self.save_image(timestamp)

                    placemark = self.kml_objects.newpoint(name=f"Marker {marker.id}")
                    placemark.timestamp.when = dt
                    placemark.coords = [(lon, lat, depth)]
                    placemark.extendeddata.newdata(name="Target ID", value=object_id)
                    placemark.extendeddata.newdata(name="Latitude", value=str(lat))
                    placemark.extendeddata.newdata(name="Longitude", value=str(lon))
                    placemark.extendeddata.newdata(name="Depth", value=str(depth))
                    placemark.extendeddata.newdata(name="Features", value=f"color={marker.color}")
                    placemark.extendeddata.newdata(name="Image", value=filename)

            # === NUMBERS ===
            for number in msg.numbers:
                if number.id not in self.seen_number_ids:
                    self.seen_number_ids.add(number.id)
                    object_found = True
                    pos = number.pose.pose.position
                    lat = pos.latitude
                    lon = pos.longitude
                    depth = -pos.altitude

                    object_id = f"number_{number.id}"
                    filename = self.save_image(timestamp)

                    placemark = self.kml_objects.newpoint(name=f"Number {number.id}")
                    placemark.timestamp.when = dt
                    placemark.coords = [(lon, lat, depth)]
                    placemark.extendeddata.newdata(name="Target ID", value=object_id)
                    placemark.extendeddata.newdata(name="Latitude", value=str(lat))
                    placemark.extendeddata.newdata(name="Longitude", value=str(lon))
                    placemark.extendeddata.newdata(name="Depth", value=str(depth))
                    placemark.extendeddata.newdata(
                        name="Features", value=f"number={number.number}, bg_color={number.bg_color}"
                    )
                    placemark.extendeddata.newdata(name="Image", value=filename)

            # === PIPES ===
            for pipe in msg.pipes:
                if pipe.id not in self.seen_pipe_ids:
                    self.seen_pipe_ids.add(pipe.id)
                    object_found = True
                    start = pipe.start_pose.pose.position
                    end = pipe.end_pose.pose.position

                    object_id = f"pipe_{pipe.id}"
                    filename = self.save_image(timestamp)

                    coords = [
                        (start.longitude, start.latitude, -start.altitude),
                        (end.longitude, end.latitude, -end.altitude)
                    ]
                    line = self.kml_objects.newlinestring(name=f"Pipe {pipe.id}")
                    line.timestamp.when = dt
                    line.coords = coords
                    line.extendeddata.newdata(name="Target ID", value=object_id)
                    line.extendeddata.newdata(name="Latitude", value=str(start.latitude))
                    line.extendeddata.newdata(name="Longitude", value=str(start.longitude))
                    line.extendeddata.newdata(name="Depth", value=str(-start.altitude))
                    line.extendeddata.newdata(name="Features", value=f"sizes={pipe.sizes}")
                    line.extendeddata.newdata(name="Image", value=filename)        
            
            if object_found:
                self.get_logger().info(f"[PERCEPTION] Logged new objects at {dt}")
            
            current_time = time.time()
            if current_time - self.last_save_time > self.save_interval:
                self.save_logs()
                self.last_save_time = current_time

        except Exception as e:
            self.get_logger().error(f"[PERCEPTION] Logging error: {e}")

            
    def save_logs(self):
        try:
            self.kml_nav.save(self.kml_path_nav)
            self.kml_mission.save(self.kml_path_mission)
            self.kml_objects.save(self.kml_path_objects)
            self.get_logger().info("KMLs saved.")
        except Exception as e:
            self.get_logger().error(f"Error saving KMLs: {e}")

    def destroy_node(self):
        self.save_logs()
        self.get_logger().info(f"Final KMLs file saved: {self.kml_path_nav}")
        super().destroy_node()