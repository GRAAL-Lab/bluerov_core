import rclpy
from rclpy.node import Node
from auv_core_helper.msg import PoseStamped, MissionStatus, DtcList
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from datetime import datetime, timedelta, timezone
from utils.utilities import STATE_NAME_MAP, TOPICS_NAMES
import os
import subprocess
import math
import simplekml
import cv2
from cv_bridge import CvBridge
import shutil
import signal
import psutil

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')        
        
        # Configuration with validation
        self.team_name = self.declare_parameter('team_name', 'UniGe_ISME').get_parameter_value().string_value
        self.max_disk_usage_gb = self.declare_parameter('max_disk_usage_gb', 5.0).get_parameter_value().double_value
        self.max_log_age_hours = self.declare_parameter('max_log_age_hours', 24.0).get_parameter_value().double_value
        
        # Validate team name
        if not self.team_name or not self.team_name.replace('_', '').isalnum():
            raise ValueError(f"Invalid team name: {self.team_name}")
        
        # Initialize state variables
        self.mission_start_time = None
        self.mission_dir = None
        self.kml_path_nav = None
        self.kml_path_mission = None
        self.kml_path_objects = None
        self.kml_nav = None
        self.kml_mission = None
        self.kml_objects = None
        self.image_save_path = None
                        
        self._shutdown_requested = False
        
        self.init_subs_and_pubs()
        
        self.save_interval = 10.0  # seconds
        self.save_timer = self.create_timer(self.save_interval, self.save_logs_callback)
        
        # Health monitoring
        self.health_check_timer = self.create_timer(60.0, self.health_check_callback)
        
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
        
        self.get_logger().info("Logger node initialized successfully")    
        
    def cleanup_resources(self):
        """Clean up all resources safely with timeout protection"""       
        self.get_logger().info("Starting cleanup...")
        
        try:
            # Stop rosbag recording first (most critical)
            self.get_logger().info("Stopping rosbag recording...")
            self.stop_rosbag_recording()
            
            # Save final logs
            self.get_logger().info("Saving final logs...")
            self.save_logs_callback()
            
            # Cancel timers
            self.get_logger().info("Canceling timers...")
            if hasattr(self, 'save_timer') and self.save_timer:
                self.save_timer.cancel()
            if hasattr(self, 'health_check_timer') and self.health_check_timer:
                self.health_check_timer.cancel()
            if hasattr(self, 'timer') and self.timer:
                self.timer.cancel()
            if hasattr(self, 'rosbag_stop_timer') and self.rosbag_stop_timer:
                self.rosbag_stop_timer.cancel()
                
            self.get_logger().info("Cleanup completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
    
    def health_check_callback(self):
        """Monitor system health and log storage"""
        try:
            # Check disk space
            if self.mission_dir:
                disk_usage = shutil.disk_usage(self.mission_dir)
                free_gb = disk_usage.free / (1024**3)
                
                if free_gb < 1.0:  # Less than 1GB free
                    self.get_logger().warn(f"Low disk space: {free_gb:.1f}GB remaining")
                    self.cleanup_old_logs()
            
            # Check memory usage
            process = psutil.Process(os.getpid())
            memory_mb = process.memory_info().rss / (1024**2)
            if memory_mb > 500:  # More than 500MB
                self.get_logger().warn(f"High memory usage: {memory_mb:.1f}MB")
            
            # Check rosbag process health
            if self.rosbag_process and self.rosbag_process.poll() is not None:
                self.get_logger().error("Rosbag process died unexpectedly")
                self.rosbag_process = None
                
        except Exception as e:
            self.get_logger().error(f"Health check failed: {e}")
    
    def cleanup_old_logs(self):
        """Remove old log directories to free space"""
        try:
            log_dir = os.path.expanduser('~/mission_logs')
            if not os.path.exists(log_dir):
                return
                
            cutoff_time = datetime.now() - timedelta(hours=self.max_log_age_hours)
            
            for item in os.listdir(log_dir):
                item_path = os.path.join(log_dir, item)
                if os.path.isdir(item_path):
                    try:
                        # Extract timestamp from folder name
                        parts = item.split('_')
                        if len(parts) >= 3:
                            timestamp_str = f"{parts[-2]}_{parts[-1]}"
                            folder_time = datetime.strptime(timestamp_str, "%Y%m%d_%H%M")
                            
                            if folder_time < cutoff_time:
                                shutil.rmtree(item_path)
                                self.get_logger().info(f"Removed old log directory: {item}")
                    except (ValueError, OSError) as e:
                        self.get_logger().debug(f"Could not process log directory {item}: {e}")
                        
        except Exception as e:
            self.get_logger().error(f"Error cleaning old logs: {e}")
        
    def create_folders_and_files(self):
        """Initialize folder structure and KML files with error handling"""
        try:
            # Create RAMI-compliant folder structure
            self.mission_start_time = datetime.now(tz=timezone.utc)
            
            # Create RAMI-compliant folder name: TEAM_X_YYYYMMDD_HHMM
            folder_timestamp = self.mission_start_time.strftime("%Y%m%d_%H%M")
            folder_name = f"{self.team_name}_{folder_timestamp}"
            
            # Create log directory with RAMI-compliant structure
            log_dir = os.path.expanduser('~/mission_logs')
            self.mission_dir = os.path.join(log_dir, folder_name)
            
            # Create directories with error handling
            os.makedirs(self.mission_dir, exist_ok=True)
            
            # Test write permissions
            test_file = os.path.join(self.mission_dir, '.write_test')
            with open(test_file, 'w') as f:
                f.write('test')
            os.remove(test_file)
            
            # Set up file paths
            self.kml_path_nav = os.path.join(self.mission_dir, "vehicle_navigation_data.kml")
            self.kml_path_mission = os.path.join(self.mission_dir, "mission_status_data.kml")
            self.kml_path_objects = os.path.join(self.mission_dir, "object_recognition_data.kml")
            self.image_save_path = os.path.join(self.mission_dir, "object_images")
            
            os.makedirs(self.image_save_path, exist_ok=True)
            
            # Initialize KML objects with thread safety            
            self.kml_nav = simplekml.Kml()
            self.kml_mission = simplekml.Kml()
            self.kml_objects = simplekml.Kml()
            
            self.get_logger().info(f"Created mission directory: {self.mission_dir}")
            self.get_logger().info(f"Logging navigation data to: {self.kml_path_nav}")
            self.get_logger().info(f"Logging mission status to: {self.kml_path_mission}")
            self.get_logger().info(f"Logging object recognition data to: {self.kml_path_objects}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to create folders and files: {e}")
            raise
        
    def init_subs_and_pubs(self):
        """Initialize subscribers and publishers with error handling"""
        try:            
            self.pose_sub = self.create_subscription(
                PoseStamped, TOPICS_NAMES["Pose"], self.pose_callback, 10)
            self.mission_sub = self.create_subscription(
                MissionStatus, TOPICS_NAMES["MissionStatus"], self.mission_callback, 10)
            self.image_sub = self.create_subscription(
                Image, TOPICS_NAMES["Camera"], self.camera_callback, rclpy.qos.qos_profile_sensor_data)
            self.detections_sub = self.create_subscription(
                DtcList, TOPICS_NAMES["Detections"], self.detection_callback, 10)

            # Publisher for downsampled image
            self.lowres_image_pub = self.create_publisher(
                Image, TOPICS_NAMES["CameraLowRes"], 10)
            
            self.last_publish_time = self.get_clock().now()
            self.publish_interval = 1.0  # seconds
            
            # Services
            self.start_srv = self.create_service(
                Trigger, '/start_logging', self.start_logging_callback)
            self.stop_srv = self.create_service(
                Trigger, '/stop_logging', self.stop_logging_callback)
            
            self.get_logger().info(f"Initialized subscriptions/publishers")
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize subscriptions/publishers: {e}")
            raise

    def start_logging_callback(self, request, response):
        """Start logging service with error handling"""
        try:
            if not self.logging_active:            
                self.create_folders_and_files()  # This can raise exceptions
                self.logging_active = True
                self.get_logger().info("[SERVICE] Logging STARTED")
                response.success = True
                response.message = "Logging started successfully"
            else:
                response.success = False
                response.message = "Logging already active"
        except Exception as e:
            self.get_logger().error(f"Failed to start logging: {e}")
            response.success = False
            response.message = f"Failed to start logging: {str(e)}"
        
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
    
    def validate_coordinates(self, lat: float, lon: float) -> bool:
        """Validate GPS coordinates"""
        if lat is None or lon is None:
            return False
        return -90 <= lat <= 90 and -180 <= lon <= 180
    
    def validate_depth(self, depth: float) -> bool:
        """Validate depth value"""
        if depth is None:
            return False
        return -1000 <= depth <= 1000  # Reasonable AUV depth range
    
    def pose_callback(self, msg: PoseStamped):
        """Pose callback with validation"""
        try:
            # Validate message
            if not hasattr(msg, 'position') or not hasattr(msg.position, 'latitude'):
                self.get_logger().debug("Invalid pose message structure")
                return
                
            # Validate coordinates
            lat = getattr(msg.position, 'latitude', None)
            lon = getattr(msg.position, 'longitude', None)
            
            if not self.validate_coordinates(lat, lon):
                self.get_logger().debug(f"Invalid coordinates: lat={lat}, lon={lon}")
                return
                
            self.latest_pose = msg
                
        except Exception as e:
            self.get_logger().error(f"Error in pose callback: {e}")
        
    def log_pose(self):
        """Log pose with robust error handling"""
        if not self.logging_active or self.latest_pose is None:
            return
            
        try:            
            pose_msg = self.latest_pose
            
            # Extract and validate timestamp
            t = pose_msg.header.stamp
            timestamp = t.sec + t.nanosec * 1e-9
            dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()

            # Extract and validate position data
            lat = pose_msg.position.latitude
            lon = pose_msg.position.longitude
            depth = getattr(pose_msg, 'depth', None)
            yaw = getattr(pose_msg, 'yaw', None)

            if not self.validate_coordinates(lat, lon):
                self.get_logger().debug("Skipping pose log due to invalid coordinates")
                return
                
            if not self.validate_depth(depth):
                self.get_logger().debug("Invalid depth value, using 0")
                depth = 0.0
                
            yaw_deg = (yaw * (180.0 / math.pi)) if yaw is not None else 0.0
            
            if self.kml_nav is not None:
                p = self.kml_nav.newpoint(name="Pose", coords=[(lon, lat)])
                p.timestamp.when = dt
                p.extendeddata.newdata(name="Heading", value=str(yaw_deg))
                p.extendeddata.newdata(name="Depth", value=str(depth))
                    
            self.get_logger().debug(f"[POSE] Logged at {dt}")
                    
        except Exception as e:
            self.get_logger().error(f"[POSE] Logging error: {e}")
    
    def mission_callback(self, msg: MissionStatus):
        """Mission status callback with validation"""
        if not self.logging_active:
            return
            
        try:
            # Validate message structure
            if not hasattr(msg, 'state'):
                self.get_logger().debug("Invalid mission status message")
                return
                
            if self.is_manipulation(msg):
                self.msn_manipulation_flag = True
            else:
                self.msn_manipulation_flag = False
                  
            self.handle_mission_status()
            
            if self.is_mission_status_changed(msg):
                self.log_mission_status(msg)          
                
        except Exception as e:
            self.get_logger().error(f"[MISSION] Callback error: {e}")
    
    def save_logs_callback(self):
        """Save logs with error handling and atomic operations"""
        if not self.logging_active:
            return
            
        try:
            if self.kml_nav and self.kml_path_nav:
                # Use temporary file for atomic save
                temp_path = self.kml_path_nav + '.tmp'
                self.kml_nav.save(temp_path)
                os.rename(temp_path, self.kml_path_nav)
                                    
            if self.kml_mission and self.kml_path_mission:
                temp_path = self.kml_path_mission + '.tmp'
                self.kml_mission.save(temp_path)
                os.rename(temp_path, self.kml_path_mission)
                    
            if self.kml_objects and self.kml_path_objects:
                temp_path = self.kml_path_objects + '.tmp'
                self.kml_objects.save(temp_path)
                os.rename(temp_path, self.kml_path_objects)
                    
            self.get_logger().debug("KML files saved successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error saving KML files: {e}")
    
    def start_rosbag_recording(self):
        """Start rosbag recording with better error handling"""
        try:
            if self.rosbag_process is None or self.rosbag_process.poll() is not None:           
                timestamp = datetime.now().strftime("%H%M%S")        
                output_dir = os.path.join(self.mission_dir, f"rosbag_{timestamp}")

                cmd = [
                    "ros2", "bag", "record",
                    TOPICS_NAMES["CameraLowRes"],
                    "-o", output_dir,
                    "--max-bag-size", "100000000",  # 100MB max bag size
                    "--compression-mode", "file"
                ]

                self.get_logger().info(f"[ROS2 BAG] Starting recording to: {output_dir}")
                
                # Start process with proper error handling
                self.rosbag_process = subprocess.Popen(
                    cmd, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Create new process group
                )
                
                # Set timer to stop after 30 seconds
                if self.rosbag_stop_timer:
                    self.rosbag_stop_timer.cancel()
                self.rosbag_stop_timer = self.create_timer(30.0, self.stop_rosbag_recording)
                
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag recording: {e}")
            self.rosbag_process = None

    def stop_rosbag_recording(self):
        """Stop rosbag recording safely"""
        try:
            self.msn_manipulation_flag = False
            self.dtc_manipulation_flag = False
            self.lowres_image_publishing = False
            
            if self.rosbag_process and self.rosbag_process.poll() is None:
                self.get_logger().info("[ROS2 BAG] Stopping rosbag recording.")
                
                # Send SIGINT to the process group
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
                
                # Wait for graceful shutdown
                try:
                    self.rosbag_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("Rosbag process didn't stop gracefully, terminating")
                    self.rosbag_process.terminate()
                    self.rosbag_process.wait(timeout=5)
                
                self.rosbag_process = None
            
            if self.rosbag_stop_timer:
                self.rosbag_stop_timer.cancel()
                self.rosbag_stop_timer = None
                
        except Exception as e:
            self.get_logger().error(f"Error stopping rosbag: {e}")

    def save_image(self, timestamp):
        """Save image with error handling"""
        if self.latest_image is None:
            self.get_logger().debug("No latest image available to save")
            return ""
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            filename = f"object_{int(timestamp * 1000)}.jpg"
            filepath = os.path.join(self.image_save_path, filename)
            
            # Save with quality control
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
            success = cv2.imwrite(filepath, cv_image, encode_params)
            
            if success:
                self.get_logger().debug(f"[IMAGE] Saved image: {filename}")
                return filename
            else:
                self.get_logger().error(f"[IMAGE] Failed to save image: {filename}")
                return ""
                
        except Exception as e:
            self.get_logger().error(f"[IMAGE] Error saving image: {e}")
            return ""

    def destroy_node(self):
        """Clean shutdown with proper resource cleanup"""
        try:
            self.get_logger().info("Shutting down logger node...")

            if self._shutdown_requested:
                return
            self._shutdown_requested = True
            self.cleanup_resources()
            
            # Cancel all timers
            if hasattr(self, 'save_timer'):
                self.save_timer.cancel()
            if hasattr(self, 'health_check_timer'):
                self.health_check_timer.cancel()
            if hasattr(self, 'timer'):
                self.timer.cancel()
                
            if self.kml_path_nav:
                self.get_logger().info(f"Final KML files saved to: {self.mission_dir}")
                
        except Exception as e:
            self.get_logger().error(f"Error during node destruction: {e}")
        finally:
            super().destroy_node()

    def is_mission_status_changed(self, msg: MissionStatus) -> bool:
        """Check if mission status has changed"""
        try:
            state = getattr(msg, 'state', None)
            state_object = getattr(msg, 'state_object', None)

            
            changed = (
                state != self.last_state or
                state_object != self.last_state_object
            )
            return changed
        except Exception as e:
            self.get_logger().error(f"Error checking mission status change: {e}")
            return False

    def is_manipulation(self, msg: MissionStatus) -> bool:
        """Check if mission is in manipulation mode"""
        try:
            state = getattr(msg, 'state', None)
            state_object = getattr(msg, 'state_object', None)
            
            if state == "Init" and state_object == "MANIPULATION":
                self.get_logger().info("[MISSION] Manipulation mode found in Init state.")
                return True
            
            return False
        except Exception as e:
            self.get_logger().error(f"Error checking manipulation mode: {e}")
            return False
    
    def handle_mission_status(self):
        """Handle mission status changes and trigger rosbag recording"""
        try:
            if self.msn_manipulation_flag and self.dtc_manipulation_flag:
                self.get_logger().info("[MISSION] Manipulation console found in teleop mode. Starting rosbag.")
                self.lowres_image_publishing = True
                self.start_rosbag_recording()
        except Exception as e:
            self.get_logger().error(f"Error handling mission status: {e}")

    def log_mission_status(self, msg: MissionStatus):
        """Log mission status to KML with error handling"""
        try:
            state = getattr(msg, 'state', None)
            state_object = getattr(msg, 'state_object', None)

            # Validate timestamp
            if not hasattr(msg, 'stamp'):
                self.get_logger().debug("Mission status message missing timestamp")
                return

            t = msg.stamp
            timestamp = t.sec + t.nanosec * 1e-9
            dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()

            readable_state = STATE_NAME_MAP.get(state, state if state else "Unknown")
            name = f"Subtask: {readable_state}"
           
            if self.kml_mission is not None:
                p = self.kml_mission.newpoint(name=name, coords=[(0, 0)])
                p.timestamp.when = dt
                p.extendeddata.newdata(name="Key Decision and Event Message", value=str(state_object) if state_object else "")

            self.get_logger().info(f"[MISSION] Logged new status '{readable_state}' at {dt}")

            # Update state tracking
            self.last_state = state
            self.last_state_object = state_object
                
        except Exception as e:
            self.get_logger().error(f"Error logging mission status: {e}")        
    
    def _log_buoy(self, buoy, dt: str, timestamp: float) -> bool:
        """Log buoy data to KML format"""
        try:
            if not hasattr(buoy, 'position'):
                return False

            pos = buoy.position
            lat = getattr(pos, 'latitude', None)
            lon = getattr(pos, 'longitude', None)
            depth = getattr(self.latest_pose, 'depth', None)

            if not self.validate_coordinates(lat, lon):
                return False
            
            if not self.validate_depth(depth):
                self.get_logger().debug("Invalid depth value, using 0")
                depth = 0.0
            
            object_id = f"buoy_{buoy.id}"
            filename = self.save_image(timestamp)  # Ensure this saves as JPEG/PNG/BMP/PPM

            if self.kml_objects is not None:
                placemark = self.kml_objects.newpoint(name=f"Buoy {buoy.id}")
                placemark.timestamp.when = dt
                placemark.coords = [(lon, lat, depth)]
                
                # Target ID
                placemark.extendeddata.newdata(name="Target ID", value=object_id)
                
                # Features
                radius = getattr(buoy, 'radius', 'unknown')
                color = getattr(buoy, 'color', 'unknown')
                color_conf = getattr(buoy, 'color_confidence', 'unknown')
                feature_info = f"radius={radius}, color={color}, confidence={color_conf}"
                placemark.extendeddata.newdata(name="Features", value=feature_info)
                
                # Image
                placemark.extendeddata.newdata(name="Image", value=filename)

            return True

        except Exception as e:
            self.get_logger().error(f"Error logging buoy: {e}")
            return False
                
    def detection_callback(self, msg):
        """Handle detection messages"""
        if not self.logging_active:
            return
        
        try:
             # Validate message structure
            if not hasattr(msg, 'header'):
                self.get_logger().debug("Invalid perception message structure")
                return
            
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()            
            
            object_found = False
            
            # === BUOYS ===
            if hasattr(msg, 'buoys'):
                for buoy in msg.buoys:
                    try:
                        if hasattr(buoy, 'id') and buoy.id not in self.seen_buoy_ids:
                            self.seen_buoy_ids.add(buoy.id)
                            object_found = True
                            
                            if self._log_buoy(buoy, dt, timestamp):
                                self.get_logger().debug(f"Logged buoy {buoy.id}")
                    
                    except Exception as e:
                        self.get_logger().error(f"Error processing buoy: {e}")                                   
            
            # === MANIPULATION CONSOLE ===
            if hasattr(msg, 'manipulation_console'):
                if msg.manipulation_console:
                    self.dtc_manipulation_flag = True
                    object_found = True
                    self.save_image(timestamp)
                    self.get_logger().info("[MISSION] Manipulation console detected.")
                    self.handle_mission_status()
                else:
                    self.dtc_manipulation_flag = False
                    
            # === RED MARKER ===
            if hasattr(msg, 'pipeline_pipe') and msg.pipeline_pipe is not None:
                pipe = msg.pipeline_pipe
                
                if hasattr(pipe, 'found_red_marker') and pipe.found_red_marker:
                    self.get_logger().info("[MISSION] Red marker found on pipe!")
                    
                    # Save image when red marker is detected
                    try:
                        # Save current frame/image
                        if hasattr(self, 'current_image') and self.current_image is not None:
                            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                            image_filename = f"red_marker_{timestamp_str}.jpg"
                            # Save image logic here
                            self.get_logger().info(f"[MISSION] Saved red marker image: {image_filename}")
                        
                        # Log red marker position
                        if hasattr(pipe, 'red_marker_position'):
                            self.get_logger().debug(f"Red marker position logged")
                            # Log the actual position coordinates
                            if self._log_red_marker(pipe.red_marker_position, dt, timestamp):
                                self.get_logger().debug("Red marker position saved to database")
                                
                    except Exception as e:
                        self.get_logger().error(f"Error saving red marker data: {e}")
                    
            if object_found:
                self.get_logger().info(f"Detected new objects at {dt}")
        
        except Exception as e:
            self.get_logger().error(f"Error in detection callback: {e}")
                    
    def convert_and_publish_lowres_image(self, msg: Image):
        """Convert and publish low-resolution image"""
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
        """Camera callback with error handling"""
        try:
            # Validate message
            if not hasattr(msg, 'header'):
                self.get_logger().debug("Invalid image message structure")
                return
                
            self.latest_image = msg  # Save latest image for perception callback
            
            if self.lowres_image_publishing:         
                self.convert_and_publish_lowres_image(msg)            

        except Exception as e:
            self.get_logger().error(f"[IMAGE] Camera callback failed: {e}")


def main(args=None):
    """Main function with proper error handling"""
    try:
        rclpy.init(args=args)
        
        logger_node = LoggerNode()
        
        try:
            rclpy.spin(logger_node)
        except KeyboardInterrupt:
            logger_node.get_logger().info("Received keyboard interrupt, shutting down...")
        finally:
            logger_node.destroy_node()
            
    except Exception as e:
        print(f"Fatal error in logger node: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()