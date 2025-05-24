#include "bluerov-bridge/bluerov_bridge.hpp"

// C / C++ Includes
#include <chrono>
#include <cmath>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

/**
 * @file bluerov_bridge.cpp
 * @brief Implementation of the BlueROVBridge class that connects ROS2 to ArduSub via MAVLink
 * 
 * Communication is via MAVLink over UDP, using the standard ArduSub protocol.
 */
BlueROVBridge::BlueROVBridge(const rclcpp::NodeOptions& options): Node("mavlink_bridge", options){
  RCLCPP_INFO(this->get_logger(), "Starting BlueROVBridge node (UDP port 14551)...");

  // Initialize the MAVLink UDP connection on local port=14551
  initMavlinkConnection();

  // Setup ROS pubs/subs
  heartBeatPublisher_ = this->create_publisher<auv_core_helper::msg::HeartBeat>(auv_core_helper::topicnames::heart_beat,1);
  globalOriginPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::global_origin,1);
  batteryStatusPublisher_ = this->create_publisher<auv_core_helper::msg::BatteryStatus>(auv_core_helper::topicnames::battery_status,1);
  globalPoseActualPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_actual_global_,1);
  globalVelocityActualPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_actual_global,1);
  dvlDistancePublisher_ = this->create_publisher<std_msgs::msg::Float64>(auv_core_helper::topicnames::dvl_distance_actual,1);

  globalPoseDesiredSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_desired_global,10,std::bind(&BlueROVBridge::globalPoseDesiredCallback, this, std::placeholders::_1));
  globalVelocityDesiredSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_desired_global,10,std::bind(&BlueROVBridge::globalVelocityDesiredCallback, this, std::placeholders::_1));

  // ROS Services
  armingService_ = this->create_service<std_srvs::srv::SetBool>(auv_core_helper::topicnames::arming_service, std::bind(&BlueROVBridge::armingServiceCallback, this,std::placeholders::_1, std::placeholders::_2));
    
  flightModeService_ = this->create_service<auv_core_helper::srv::SetFlightMode>(auv_core_helper::topicnames::flight_mode_service,std::bind(&BlueROVBridge::flightModeServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Timers
  data_timer_ = this->create_wall_timer(std::chrono::milliseconds(125), std::bind(&BlueROVBridge::receiveData, this)); // ~8Hz
  mainTimer_ = this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&BlueROVBridge::Execute, this)); // ~8Hz
  

  poseGoalGlobal.setZero();
  poseGoalGlobalLast.setConstant(std::numeric_limits<double>::quiet_NaN());
  velocityDesiredGlobal.setZero();

}

BlueROVBridge::~BlueROVBridge(){
  if (sock_fd_ != -1) {
    close(sock_fd_);
  }
  RCLCPP_INFO(this->get_logger(), "BlueROVBridge node shutting down.");
}

/**
 * @brief Initialize MAVLink UDP connection
 * 
 * This method:
 * 1. Creates a UDP socket bound to port 14551 (standard ArduSub port)
 * 2. Sets up initial remote_addr_ to the simulation address (127.0.0.1)
 * 3. Will update remote_addr_ when the first heartbeat is received
 * 
 * @throws std::runtime_error if socket creation or binding fails
 */
void BlueROVBridge::initMavlinkConnection(){
  // Create UDP socket
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    const std::string err_msg = "Failed to create UDP socket: " + std::string(strerror(errno));
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    throw std::runtime_error(err_msg);
  }

  // Set up socket options - allow port reuse
  int reuse = 1;
  if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to set SO_REUSEADDR: %s", strerror(errno));
  }

  // Configure local address to bind to all interfaces (0.0.0.0) on port 14551
  sockaddr_in local_addr;
  std::memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family      = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY; // 0.0.0.0 (listen on all interfaces)
  local_addr.sin_port        = htons(14551);  // Match BlueOS MAVLink endpoint

  // Bind the socket
  if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
    const std::string err_msg = "Socket bind failed on port 14551: " + std::string(strerror(errno));
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    close(sock_fd_);
    throw std::runtime_error(err_msg);
  }

  RCLCPP_INFO(this->get_logger(),
      "Bound to 0.0.0.0:14551, waiting for ArduSub telemetry (will initially send to 127.0.0.1:14551)");

  // Initialize remote address (will be updated when first heartbeat is received)
  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");  // Default for simulation
  remote_addr_.sin_port = htons(14551); // Send commands to ArduPilot on 14551

  // Initialize ArduPilot-related fields
  target_system_   = 0;
  target_component_= 0;
  got_heartbeat_   = false;
}

void BlueROVBridge::receiveData(){
  while (true)  // Keep reading until no more data is available
  {
    uint8_t buffer[2048];
    sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    // Non-blocking recv
    ssize_t recsize = recvfrom(
        sock_fd_,
        buffer,
        sizeof(buffer),
        MSG_DONTWAIT,
        reinterpret_cast<struct sockaddr*>(&sender_addr),
        &addr_len
    );

    // If there's no more data (or an error), break out of the while loop
    if (recsize <= 0) {
      break;
    }

    mavlink_message_t msg;
    mavlink_status_t status;

    // Parse all bytes in the received packet
    for (ssize_t i = 0; i < recsize; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
        // Route message to appropriate handler based on msg.msgid
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg, sender_addr);
            break;
            
          case MAVLINK_MSG_ID_ATTITUDE:
            handleAttitude(msg);
            break;
            

          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            handleGlobalPositionInt(msg);
            break;  
          
          case MAVLINK_MSG_ID_DISTANCE_SENSOR:  
            handleDvlDistance(msg);
            break;
            
          case MAVLINK_MSG_ID_COMMAND_ACK:
            handleCommandAck(msg);
            break;

          case MAVLINK_MSG_ID_BATTERY_STATUS:
            handleBatteryStatus(msg);
            break;

          case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
            handleGlobalOrigin(msg);
            break;
            
          default:
            break;
        }
      }
    }
  }
}

void BlueROVBridge::sendMavlinkMessage(const mavlink_message_t& msg){
  if (sock_fd_ < 0) {
    return;
  }

  uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t msg_len = mavlink_msg_to_send_buffer(tx_buffer, &msg);

  ssize_t bytes_sent = sendto(
      sock_fd_,
      tx_buffer,
      msg_len,
      0,
      reinterpret_cast<const struct sockaddr*>(&remote_addr_),
      sizeof(remote_addr_)
  );
  if (bytes_sent < 0) {
    RCLCPP_ERROR(this->get_logger(),
        "Failed sending MAVLink message (errno=%d).", errno);
  }
}

void BlueROVBridge::setMessageInterval(uint16_t message_id, float frequency_hz){
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(),
        "setMessageInterval(%d) called, but no autopilot heartbeat yet!", message_id);
    return;
  }

  // Interval in microseconds
  float interval_us = 1.0e6f / frequency_hz;  

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_SET_MESSAGE_INTERVAL,
      0, // confirmation
      static_cast<float>(message_id), // param1
      interval_us,                    // param2
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f    // param3..7 unused
  );

  sendMavlinkMessage(msg);
  RCLCPP_INFO(this->get_logger(),
      "Requested message #%d at %.1f Hz (%.0f us).",
      message_id, frequency_hz, interval_us);
}

void BlueROVBridge::handleHeartbeat(const mavlink_message_t& msg, const sockaddr_in& sender_addr){
  // Ignore if it's our own GCS heartbeat
  if (msg.sysid == system_id_ && msg.compid == component_id_) {
    RCLCPP_INFO(this->get_logger(),
       "Ignoring GCS heartbeat (sys=%d, comp=%d).", msg.sysid, msg.compid);
    return;
  }

  // Extract the heartbeat info
  mavlink_msg_heartbeat_decode(&msg, &hb);

  auto heartBeatMsg = std::make_unique<auv_core_helper::msg::HeartBeat>();
  heartBeatMsg->type = hb.type;
  heartBeatMsg->base_mode = hb.base_mode;
  heartBeatMsg->custom_mode = hb.custom_mode;
  heartBeatMsg->system_status = hb.system_status;

  heartBeatPublisher_->publish(*heartBeatMsg);

  if (!got_heartbeat_) {
    target_system_    = msg.sysid;
    target_component_ = msg.compid;
    got_heartbeat_    = true;

    // Overwrite remote_addr_ with the sender's IP:port
    remote_addr_ = sender_addr;

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(sender_addr.sin_addr), ip_str, sizeof(ip_str));
    uint16_t sender_port = ntohs(sender_addr.sin_port);

    RCLCPP_INFO(this->get_logger(),
        "Got AUTOPILOT heartbeat from sys=%d, comp=%d at %s:%d => storing remote_addr_",
        target_system_, target_component_, ip_str, sender_port);

    // Configure data streams directly 
    setMessageInterval(MAVLINK_MSG_ID_ATTITUDE, 8.0f);            // #30
    setMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 8.0f); // #33
    setMessageInterval(MAVLINK_MSG_ID_DISTANCE_SENSOR, 8.0f);     // #34
    setMessageInterval(MAVLINK_MSG_ID_COMMAND_ACK, 8.0f);         // #35
    setMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, 8.0f);      // #147
    setMessageInterval(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 8.0f);   // #32
  }
}

void BlueROVBridge::handleGlobalOrigin(const mavlink_message_t& msg){
  mavlink_gps_global_origin_t gps_global_origin;
  mavlink_msg_gps_global_origin_decode(&msg, &gps_global_origin);
  
  auto global_origin_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
  global_origin_msg->header.stamp = this->now();
  global_origin_msg->header.frame_id = "Global WGS84";
  global_origin_msg->position.latitude = gps_global_origin.latitude / 1e7;  // Convert to degrees 
  global_origin_msg->position.longitude = gps_global_origin.longitude / 1e7;  // Convert to degrees
  global_origin_msg->depth = gps_global_origin.altitude / 1000.0;  // mm → meters
  globalOriginPublisher_->publish(*global_origin_msg);
}

void BlueROVBridge::handleBatteryStatus(const mavlink_message_t& msg){
   mavlink_battery_status_t battery_status;
   mavlink_msg_battery_status_decode(&msg, &battery_status);

   auto battery_status_ = std::make_unique<auv_core_helper::msg::BatteryStatus>();
   battery_status_->temperature = battery_status.temperature;
   for (size_t i = 0; i < 10; i++) {
     battery_status_->voltages[i] = battery_status.voltages[i];
   }
   battery_status_->current_battery = battery_status.current_battery;
   battery_status_->current_consumed = battery_status.current_consumed;
   battery_status_->energy_consumed = battery_status.energy_consumed;
   battery_status_->battery_percentage = battery_status.battery_remaining;

   batteryStatusPublisher_->publish(*battery_status_);
 }


void BlueROVBridge::handleGlobalPositionInt(const mavlink_message_t& msg){
  mavlink_global_position_int_t pos_int;
  mavlink_msg_global_position_int_decode(&msg, &pos_int);

  global_pose_msg->header.stamp = this->now();
  global_pose_msg->header.frame_id = "Global WGS84";
  global_pose_msg->position.latitude = pos_int.lat / 1e7;  // Convert to degrees 
  global_pose_msg->position.longitude = pos_int.lon / 1e7;  // Convert to degrees
  global_pose_msg->depth = -pos_int.alt / 1000.0;  // mm → meters

  global_velocity_msg->linear.x = pos_int.vx / 100.0;  // cm/s → m/s
  global_velocity_msg->linear.y = pos_int.vy / 100.0;
  global_velocity_msg->linear.z = pos_int.vz / 100.0;
}

void BlueROVBridge::handleAttitude(const mavlink_message_t& msg){
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(&msg, &attitude);
  
  global_pose_msg->roll = attitude.roll;            //Roll in rad
  global_pose_msg->pitch = attitude.pitch;         //Pitch in rad
  global_pose_msg->yaw = attitude.yaw;             //Yaw in rad

  global_velocity_msg->angular.x = attitude.rollspeed;          //Roll rate in rad/s
  global_velocity_msg->angular.y = attitude.pitchspeed;        //Pitch rate in rad/s
  global_velocity_msg->angular.z = attitude.yawspeed;           //Yaw rate in rad/s
  
}

void BlueROVBridge::Execute(){
  if (global_pose_msg && global_velocity_msg && got_heartbeat_) {
    globalPoseActualPublisher_->publish(*global_pose_msg);
    globalVelocityActualPublisher_->publish(*global_velocity_msg);
    if(flightMode_actual == "GUIDED"){
      //Pose controllr
      if (poseGoalGlobalChanged){
        condition_yaw_.target_system = target_system_;
        condition_yaw_.target_component = target_component_;
        condition_yaw_.param1 = poseGoalGlobal(5)*180.0f/M_PI; 

        sendConditionYaw(condition_yaw_);


        position_target_global_.time_boot_ms     = static_cast<uint32_t>(this->now().nanoseconds() / 1e6);
        position_target_global_.target_system    = target_system_;
        position_target_global_.target_component = target_component_;
        position_target_global_.coordinate_frame = MAV_FRAME_GLOBAL_INT;            
        position_target_global_.type_mask =
            POSITION_TARGET_TYPEMASK_VX_IGNORE  |
            POSITION_TARGET_TYPEMASK_VY_IGNORE  |
            POSITION_TARGET_TYPEMASK_VZ_IGNORE  |
            POSITION_TARGET_TYPEMASK_AX_IGNORE  |
            POSITION_TARGET_TYPEMASK_AY_IGNORE  |
            POSITION_TARGET_TYPEMASK_AZ_IGNORE  |
            POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
        position_target_global_.lat_int = static_cast<int32_t>(poseGoalGlobal(0) * 1e7);       // deg → 1e-7°
        position_target_global_.lon_int = static_cast<int32_t>(poseGoalGlobal(1) * 1e7);
        position_target_global_.alt     = poseGoalGlobal(2);

        position_target_global_.yaw      = static_cast<float>(poseGoalGlobal(5));
        position_target_global_.yaw_rate = 0.0f;

        SetPositionTargetGlobalInt(position_target_global_);
        poseGoalGlobalLast = poseGoalGlobal;
        poseGoalGlobalChanged     = false;
      }
    }
    else{
      //Velocity Controller
        position_target_global_.vx = velocityDesiredGlobal(0);
        position_target_global_.vy = velocityDesiredGlobal(1);
        position_target_global_.vz = velocityDesiredGlobal(2);
        position_target_global_.yaw_rate = velocityDesiredGlobal(5);
        SetPositionTargetGlobalInt(position_target_global_);
    }
    

  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for MAVLink global position/velocity data...");
  }
}

void BlueROVBridge::handleDvlDistance(const mavlink_message_t& msg){
  mavlink_distance_sensor_t distance_sensor;
  mavlink_msg_distance_sensor_decode(&msg, &distance_sensor);

  auto dvl_distance_msg = std::make_unique<std_msgs::msg::Float64>();
  dvl_distance_msg->data = distance_sensor.current_distance;  //Distance in cm 

  dvlDistancePublisher_->publish(std::move(dvl_distance_msg));
}

void BlueROVBridge::handleCommandAck(const mavlink_message_t& msg){
  mavlink_msg_command_ack_decode(&msg, &ack);

}

void BlueROVBridge::armingServiceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
  RCLCPP_INFO(this->get_logger(), "Arming service called: %s", request->data ? "ARM" : "DISARM");
  setArmState(request->data);                                     // True to arm, false to disarm
  if(ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
    if (ack.result == MAV_RESULT_ACCEPTED) {
      response->success = true;
      response->message = request->data ? "Arm command sent." : "Disarm command sent.";  
    } else {
      const char* error_str = "Unknown";
      switch (ack.result) {
        case MAV_RESULT_DENIED: error_str = "Arming vehicle DENIED"; break;
        case MAV_RESULT_FAILED: error_str = "Arming vehicle FAILED"; break;
        case MAV_RESULT_TEMPORARILY_REJECTED: error_str = "Arming vehicle TEMPORARILY_REJECTED"; break;
        case MAV_RESULT_CANCELLED: error_str = "Arming vehicle CANCELLED"; break;
        default: error_str = "UNKNOWN"; break;
        response->success = false;
        response->message = error_str;
      }
      RCLCPP_WARN(this->get_logger(), "Arming vehicle FAILED: %s", error_str);
    }
  }
}

void BlueROVBridge::flightModeServiceCallback(const std::shared_ptr<auv_core_helper::srv::SetFlightMode::Request> request,std::shared_ptr<auv_core_helper::srv::SetFlightMode::Response> response){
  // RCLCPP_INFO(this->get_logger(), "Flight mode service called: %s", request->mode.c_str());
  if (ack.command == MAV_CMD_DO_SET_MODE) {
    if (ack.result == MAV_RESULT_ACCEPTED) {
      response->success = true;
      response->message = "Flight mode command sent.";
    } else {
      const char* error_str = "Unknown";
      switch (ack.result) {
        case MAV_RESULT_DENIED: error_str = "Setting Flight Mode DENIED"; break;
        case MAV_RESULT_UNSUPPORTED: error_str = "Flight Mode UNSUPPORTED"; break;
        case MAV_RESULT_FAILED: error_str = "Setting Flight Mode FAILED"; break;
        case MAV_RESULT_TEMPORARILY_REJECTED: error_str = "Setting Flight Mode TEMPORARILY_REJECTED"; break;
        default: error_str = "UNKNOWN"; break;
      }
      response->success = false;
      response->message = error_str;
      RCLCPP_WARN(this->get_logger(), "Setting Flight Mode FAILED: %s", error_str);
    }
  }
  flightMode_actual = request->mode;
  setFlightMode(request->mode);
}

void BlueROVBridge::setArmState(bool arm_vehicle)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot %s yet; no autopilot heartbeat discovered!", arm_vehicle ? "arm" : "disarm");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
      "%s vehicle (sys=%d, comp=%d)...",
      arm_vehicle ? "Arming" : "Disarming",
      target_system_, target_component_);

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,                            // Confirmation
      arm_vehicle ? 1.0f : 0.0f,    // param1: 1 to arm, 0 to disarm
      0.0f,                         // param2: Force arm/disarm (0=normal)
      0,0,0,0,0                     // Unused parameters
  );
  sendMavlinkMessage(msg);
  RCLCPP_INFO(this->get_logger(), "%s command sent.", arm_vehicle ? "Arm" : "Disarm");
}

void BlueROVBridge::setFlightMode(const std::string& mode)
{
  // Map mode string to ArduSub custom mode number
  int32_t custom_mode = 19;         // Default to MANUAL=19
  if (mode == "MANUAL") {           // Pass-through input with no stabilization
    custom_mode = 19;
  } else if (mode == "STABILIZE") { // manual angle with manual depth/throttle
    custom_mode = 0;
  } else if (mode == "ALT_HOLD") {  // manual body-frame angular rate with manual depth/throttle
    custom_mode = 2;
  } else if (mode == "GUIDED") {    // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    custom_mode = 4; 
  } else if (mode == "POSHOLD") {   // automatic position hold with manual override, with automatic throttle
    custom_mode = 16;
  } else if (mode == "SURFACE") {   // automatically return to surface, pilot maintains horizontal control
    custom_mode = 9;
  } else if (mode == "SURFTRAK") {  // Track distance above seafloor (hold range)
    custom_mode = 21 ;
  }
  else {
    RCLCPP_WARN(this->get_logger(), 
        "Unknown mode '%s', defaulting to MANUAL", mode.c_str());
  }
  
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_DO_SET_MODE,
      0, // confirmation
      1,  // param1: Mode, as defined by MAV_MODE enum (1 = MODE_GUIDED)
      static_cast<float>(custom_mode),  // param2: Custom mode - ArduSub mode
      0,  // param3: Custom sub-mode - not used for ArduSub
      0, 0, 0, 0 // param4-7 unused
  );

  sendMavlinkMessage(msg);
  // RCLCPP_INFO(this->get_logger(), "Flight mode set to %s", mode.c_str());
}

void BlueROVBridge::globalPoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg)
{
  poseGoalGlobal << msg->position.latitude,
           msg->position.longitude,
           msg->depth,
           msg->roll,
           msg->pitch,
           msg->yaw;

  auto almost_equal = [this](double a, double b, double eps){
      return std::fabs(a - b) < eps;
  };

  poseGoalGlobalChanged =
        !almost_equal(poseGoalGlobal(0), poseGoalGlobalLast(0), LAT_LON_EPS) ||
        !almost_equal(poseGoalGlobal(1), poseGoalGlobalLast(1), LAT_LON_EPS) ||
        !almost_equal(poseGoalGlobal(2), poseGoalGlobalLast(2), DEPTH_EPS)   ||
        !almost_equal(poseGoalGlobal(5), poseGoalGlobalLast(5), YAW_EPS);

}


void BlueROVBridge::globalVelocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
  velocityDesiredGlobal << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;  
}

void BlueROVBridge::SetPositionTargetGlobalInt(const mavlink_set_position_target_global_int_t& position_target_global_)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send global waypoint; no autopilot heartbeat discovered!");
    return;
  }
    
  mavlink_message_t msg;
  mavlink_msg_set_position_target_global_int_encode(
      system_id_,
      component_id_,
      &msg,
      &position_target_global_
  );
  
  sendMavlinkMessage(msg);
}

void BlueROVBridge::SetAttitudeTarget(const mavlink_set_attitude_target_t& attitude_target_)
{

  mavlink_message_t msg;
  mavlink_msg_set_attitude_target_pack(
      system_id_,
      component_id_,
      &msg,
      attitude_target_.time_boot_ms,
      attitude_target_.target_system,
      attitude_target_.target_component,
      attitude_target_.type_mask, 
      attitude_target_.q, 
      0,           // body_roll_rate (not used)
      0,           // body_pitch_rate (not used)
      0,           // body_yaw_rate (not used)
      0,           // thrust (not used)
      0            // thrust_body (not used) 
  );
  
  sendMavlinkMessage(msg);
  RCLCPP_INFO(this->get_logger(), "ATTITUDE_TARGET command sent");
}

void BlueROVBridge::sendConditionYaw(const mavlink_command_long_t& condition_yaw_)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send CONDITION_YAW command; no autopilot heartbeat discovered!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(),
      "Setting vehicle heading: %.1f degrees",
      condition_yaw_.param1);
    
  // Create command message
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_CONDITION_YAW,
      0.0f,                       // confirmation field 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
      condition_yaw_.param1,      // param1: target angle (degrees)
      0.0f,                       // param2: angular speed (deg/sec) ArduPilot interprets yaw rate = 0 as: Use the default yaw rate defined in the firmware parameters.
      0.0f,                       // param3: direction: -1=CCW, 1=CW, 0=shortest
      0.0f,                       // param4: 0=absolute, 1=relative
      0.0f, 0.0f, 0.0f);         // param5-7: unused
  
  // Send the message
  sendMavlinkMessage(msg);
  RCLCPP_INFO(this->get_logger(), "CONDITION_YAW command sent");
}
