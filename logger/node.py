import rclpy
from rclpy.node import Node
from auv_core_helper.msg import PoseStamped, MissionStatus, DtcList
from image_pipeline_msgs.msg import Obstacles
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from datetime import datetime, timezone
from utils.utilities import STATE_NAME_MAP, TOPICS_NAMES
import os
import subprocess
import math
import simplekml
import cv2
from cv_bridge import CvBridge

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')        
        
        self.team_name = self.declare_parameter('team_name', 'UniGe_ISME').get_parameter_value().string_value
        self.mission_start_time = None
        self.mission_dir = None
        self.kml_path_nav = None
        self.kml_path_mission = None
        self.kml_path_objects = None
        self.kml_nav = None
        self.kml_mission = None
        self.kml_objects = None
        self.image_save_path = None
        
        self.init_subs_and_pubs()
        #self.create_folders_and_files()                
        
        self.save_interval = 10.0  # seconds
        self.save_timer = self.create_timer(self.save_interval, self.save_logs_callback)
        
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
        self.msn_manipulation_flag = False
        self.dtc_manipulation_flag = False
        self.lowres_image_publishing = False
        self.logging_active = False
        
        # Initialize rosbag process handle
        self.rosbag_process = None
        self.rosbag_stop_timer = None

        # Log pose at 1 Hz
        self.timer = self.create_timer(1.0, self.log_pose)    
        
        
    def create_folders_and_files(self):
        """        Initializes the folder structure and KML files for logging mission data.
        Creates a RAMI-compliant folder structure based on the team name and mission start time.
        Creates KML files for navigation, mission status, and object recognition data.
        Also creates a directory for saving images of recognized objects.
        """
        
        # create RAMI-compliant folder structure
        self.mission_start_time = datetime.now(tz=timezone.utc)
        
        # Create RAMI-compliant folder name: TEAM_X_YYYYMMDD_HHMM
        folder_timestamp = self.mission_start_time.strftime("%Y%m%d_%H%M")
        folder_name = f"{self.team_name}_{folder_timestamp}"
        
        # Create log directory with RAMI-compliant structure
        log_dir = os.path.expanduser('~/mission_logs')
        self.mission_dir = os.path.join(log_dir, folder_name)
        os.makedirs(self.mission_dir, exist_ok=True)
        
        self.kml_path_nav = os.path.join(self.mission_dir, "vehicle_navigation_data.kml")
        self.kml_path_mission = os.path.join(self.mission_dir, "mission_status_data.kml")
        self.kml_path_objects = os.path.join(self.mission_dir, "object_recognition_data.kml")
        self.image_save_path = os.path.join(self.mission_dir, "object_images")
        os.makedirs(self.image_save_path, exist_ok=True)
        
        self.kml_nav = simplekml.Kml()
        self.kml_mission = simplekml.Kml()
        self.kml_objects = simplekml.Kml()
        
        self.get_logger().info(f"Logging navigation data to: {self.kml_path_nav}")
        self.get_logger().info(f"Logging mission status to: {self.kml_path_mission}")
        self.get_logger().info(f"Logging object recognition data to: {self.kml_path_objects}")
        
        
    def init_subs_and_pubs(self):
        # subscribers
        self.pose_sub = self.create_subscription(PoseStamped, TOPICS_NAMES["Pose"], self.pose_callback, 10)
        self.mission_sub = self.create_subscription(MissionStatus, TOPICS_NAMES["MissionStatus"], self.mission_callback, 10)
        self.perception_sub = self.create_subscription(Obstacles, TOPICS_NAMES["Obstacles"],self.perception_callback,10)
        self.image_sub = self.create_subscription(Image, TOPICS_NAMES["Camera"], self.camera_callback, rclpy.qos.qos_profile_sensor_data) # Subscribe to original high-res image topic
        self.detections_sub = self.create_subscription(DtcList, TOPICS_NAMES["Detections"], self.detection_callback, 10) # Subscribe to original high-res image topic

        # Publisher for downsampled image
        self.lowres_image_pub = self.create_publisher(Image, TOPICS_NAMES["CameraLowRes"], 10)
        # Optional: Limit publish rate
        self.last_publish_time = self.get_clock().now()
        self.publish_interval = 1.0  # seconds (adjust as needed)
        
        #service
        self.start_srv = self.create_service(Trigger, '/start_logging', self.start_logging_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_logging', self.stop_logging_callback)

    def start_logging_callback(self, request, response):
        if not self.logging_active:            
            self.logging_active = True
            self.get_logger().info(f"[SERVICE] Logging STARTED")
            response.success = True
            self.create_folders_and_files()
            response.message = f"Logging started"
        else:
            response.success = False
            response.message = "Logging already active"
        return response

    def stop_logging_callback(self, request, response):
        if self.logging_active:
            self.logging_active = False
            self.get_logger().info(f"[SERVICE] Logging STOPPED")
            response.success = True
            response.message = "Logging stopped successfully"
        else:
            response.success = False
            response.message = "Logging was not active"
        return response
    
    # Pose - Vehicle Navigation Data ##########################################
    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg        
        
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

            if lat is not None and lon is not None:
                p = self.kml_nav.newpoint(name="Pose", coords=[(lon, lat)])
                p.timestamp.when = dt
                p.extendeddata.newdata(name="Heading", value=str(yaw_deg))
                p.extendeddata.newdata(name="Depth", value=str(depth))
                self.get_logger().info(f"[POSE] Logged at {dt}")
            else:
                self.get_logger().warn("[POSE] Latitude or longitude is None, skipping KML log.")
                    
        except Exception as e:
            self.get_logger().error(f"[POSE] Logging error: {e}")
    
    # Mission Status Data ##########################################
    def mission_callback(self, msg: MissionStatus):
        if not self.logging_active:
            return
        try:
            if self.is_manipulation(msg):
                self.msn_manipulation_flag = True
            else:
                self.msn_manipulation_flag = False
                  
            self.handle_mission_status()
            
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

    def is_manipulation(self, msg: MissionStatus) -> bool:
        state = getattr(msg, 'state', None)
        state_object = getattr(msg, 'state_object', None)
        
        if state == "Init" and state_object == "MANIPULATION":
            self.get_logger().info("[MISSION] Manipulation mode found in Init state.")
            return True
        
        return False
    
    def handle_mission_status(self):
        if self.msn_manipulation_flag and self.dtc_manipulation_flag:
            self.get_logger().info("[MISSION] manipulation console found in teleop mod. Starting rosbag.")
            self.lowres_image_publishing = True
            self.start_rosbag_recording()        

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
        
        
    # Perception - Object Recognition Data ##########################################
    def perception_callback(self, msg):
        if not self.logging_active:
            return
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()
            
            # Skip if no image available yet
            if self.latest_image is None:
                self.get_logger().warning("[IMAGE] No latest image available to save yet, skipping saving image.")
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
                    line.extendeddata.newdata(name="Features", value=f"sizes={pipe.sizes}")
                    line.extendeddata.newdata(name="Image", value=filename)        
            
            if msg.manipulation_console:
                filename = self.save_image(timestamp)
                
            if object_found:
                self.get_logger().info(f"[PERCEPTION] Logged new objects at {dt}")
                object_found = False         

        except Exception as e:
            self.get_logger().error(f"[PERCEPTION] Logging error: {e}")
    
    def detection_callback(self, msg):
        if msg.manipulation_console:
            self.dtc_manipulation_flag = True
            self.handle_mission_status()
        else:
            self.dtc_manipulation_flag = False
    
    def convert_and_publish_lowres_image(self, msg: Image):
        try:            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Downscale image (e.g., 640x480)
            downscaled = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_LINEAR)

            # Limit publish rate
            now = self.get_clock().now()
            if (now - self.last_publish_time).nanoseconds / 1e9 >= self.publish_interval:
                self.last_publish_time = now

                # Convert back to ROS image
                lowres_msg = self.bridge.cv2_to_imgmsg(downscaled, encoding='bgr8')
                lowres_msg.header = msg.header
                self.lowres_image_pub.publish(lowres_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
            
    def camera_callback(self, msg: Image):
        try:
            self.latest_image = msg # Save latest image for perception callback
            if self.lowres_image_publishing:         
                self.convert_and_publish_lowres_image(msg)            

        except Exception as e:
            self.get_logger().error(f"[IMAGE] Image callback failed: {e}")            
    
    def save_image(self, timestamp):
        if self.latest_image is None:
            self.get_logger().warning("[IMAGE] No latest image available to save")
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
    
    # Start and stop rosbag recording methods ##########################################
    def start_rosbag_recording(self):
        if self.rosbag_process is None or self.rosbag_process.poll() is not None:           
            timestamp = datetime.now().strftime("%H%M%S")        
            output_dir = os.path.join(self.mission_dir, "rosbag") + f"_{timestamp}"

            cmd = [
                "ros2", "bag", "record",
                TOPICS_NAMES["CameraLowRes"],
                "-o", output_dir
            ]

            self.get_logger().info(f"[ROS2 BAG] Starting recording to: {output_dir}")
            self.rosbag_process = subprocess.Popen(cmd)
            
            # Set timer to stop after 30 seconds
            if self.rosbag_stop_timer:
                self.rosbag_stop_timer.cancel()
            self.rosbag_stop_timer = self.create_timer(30.0, self.stop_rosbag_recording)

    def stop_rosbag_recording(self):
        self.msn_manipulation_flag = False
        self.dtc_manipulation_flag = False
        self.lowres_image_publishing = False
        
        if self.rosbag_process and self.rosbag_process.poll() is None:
            self.get_logger().info("[ROS2 BAG] Stopping rosbag recording.")
            self.rosbag_process.terminate()
            self.rosbag_process.wait()
            self.rosbag_process = None
        
        if self.rosbag_stop_timer:
            self.rosbag_stop_timer.cancel()
            self.rosbag_stop_timer = None       
    
    # Save logs periodically ##########################################     
            
    def save_logs_callback(self):
        if not self.logging_active:
            return     
        try:
            self.kml_nav.save(self.kml_path_nav)
            self.kml_mission.save(self.kml_path_mission)
            self.kml_objects.save(self.kml_path_objects)
            self.get_logger().info("KMLs saved.")
        except Exception as e:
            self.get_logger().error(f"Error saving KMLs: {e}")

    def destroy_node(self):
        """
        Cleanly shuts down the node by stopping rosbag recording, saving KML logs, 
        and calling the parent class's destroy_node method.
        """
        self.stop_rosbag_recording()
        self.save_logs_callback()
        self.get_logger().info(f"Final KMLs file saved: {self.kml_path_nav}")
        super().destroy_node()