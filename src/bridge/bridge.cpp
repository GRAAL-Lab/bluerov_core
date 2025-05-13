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

//=============================================================================
// Constructor
//=============================================================================
BlueROVBridge::BlueROVBridge(const rclcpp::NodeOptions& options)
  : Node("mavlink_bridge", options)
{
  RCLCPP_INFO(this->get_logger(), 
      "Starting BlueROVBridge node (UDP port 14551)...");

  // Initialize the MAVLink UDP connection on local port=14551
  initMavlinkConnection();

  // Setup ROS pubs/subs
  heartBeatPublisher_ = this->create_publisher<auv_core_helper::msg::HeartBeat>(auv_core_helper::topicnames::heart_beat,10);// check on the quque (10)
  localPoseActualPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::local_pose_actual,10);
  globalPoseActualPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::global_pose_actual,10);
  localVelocityActualPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::local_velocity_actual,10);
  globalVelocityActualPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::global_velocity_actual,10);
  dvlDistancePublisher_ = this->create_publisher<std_msgs::msg::Float64>(auv_core_helper::topicnames::dvl_distance_actual,10);

  // Add battery status publisher
  // Always publish somesort of error message / do not reject internally without a message to inform other nodes
  // UDP connection with a managing logic for quality is a good idea
  // Recheck on the syntax of the code following the convention of the lab 
  // Name of the variables should be more descriptive (the name of the variables explicitly have the name of the frame represented in)
  // use standarized units internally between the nodes and tell marco in the gui or loging to convert to human readable units
  // Ask youssef how the hardware is set (do we need to have a controll on the camera)
  // bitmask is internal to the node and not exposed to the outside

  localPoseDesiredSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::local_pose_desired,10,
    std::bind(&BlueROVBridge::localPoseDesiredCallback, this, std::placeholders::_1));
  localVelocityDesiredSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::local_velocity_desired,10,
    std::bind(&BlueROVBridge::velocityDesiredCallback, this, std::placeholders::_1));
  rcChannelValuesDesiredSubscription_ = this->create_subscription<auv_core_helper::msg::RCChannels>(auv_core_helper::topicnames::rc_channel_values_desired,10,
    std::bind(&BlueROVBridge::rcChannelValuesDesiredCallback, this, std::placeholders::_1));
  
  // ROS Services
  armingService_ = this->create_service<std_srvs::srv::SetBool>(auv_core_helper::topicnames::arming_service, std::bind(&BlueROVBridge::armingServiceCallback, this,
     std::placeholders::_1, std::placeholders::_2));
    
  flightModeService_ = this->create_service<auv_core_helper::srv::SetFlightMode>(auv_core_helper::topicnames::flight_mode_service,
     std::bind(&BlueROVBridge::flightModeServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Timers
  data_timer_ = this->create_wall_timer(std::chrono::milliseconds(125), std::bind(&BlueROVBridge::receiveData, this)); // ~8Hz

}

//=============================================================================
// Destructor
//=============================================================================
BlueROVBridge::~BlueROVBridge()
{
  if (sock_fd_ != -1) {
    close(sock_fd_);
  }
  RCLCPP_INFO(this->get_logger(), "BlueROVBridge node shutting down.");
}

//=============================================================================
// initMavlinkConnection
//   Bind local port=14551. We do not set remote_addr_ until we see autopilot heartbeat.
//=============================================================================
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
void BlueROVBridge::initMavlinkConnection()
{
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

//=============================================================================
// receiveData()
//   Non-blocking read; parse MAVLink. Once we see autopilot's heartbeat, store
//   that IP/port in remote_addr_, so we can send commands back on same port.
//=============================================================================
void BlueROVBridge::receiveData()
{
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
            
          case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            handleLocalPositionNed(msg);
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
            
          default:
            break;
        }
      }
    }
  }
}

//=============================================================================
// sendMavlinkMessage
//=============================================================================
void BlueROVBridge::sendMavlinkMessage(const mavlink_message_t& msg)
{
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

//=============================================================================
// setMessageInterval
//=============================================================================
void BlueROVBridge::setMessageInterval(uint16_t message_id, float frequency_hz)
{
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

//=============================================================================
// Message handler functions
// Heartbeat
//=============================================================================

void BlueROVBridge::handleHeartbeat(const mavlink_message_t& msg, const sockaddr_in& sender_addr)
{
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
    setMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 8.0f);  // #32
    setMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 8.0f); // #33
    setMessageInterval(MAVLINK_MSG_ID_DISTANCE_SENSOR, 8.0f);     // #34
    setMessageInterval(MAVLINK_MSG_ID_COMMAND_ACK, 8.0f);         // #35

    RCLCPP_INFO(this->get_logger(),
        "Configured messages interval at 8 Hz.");
  }
}

//=============================================================================
// Handler for Publishing Position and Velocity in Local NED Frame
//=============================================================================
void BlueROVBridge::handleLocalPositionNed(const mavlink_message_t& msg)
{
  mavlink_local_position_ned_t pos_ned;
  mavlink_msg_local_position_ned_decode(&msg, &pos_ned);

  auto local_pose_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
  local_pose_msg->header.stamp = this->now();
  local_pose_msg->header.frame_id = "local NED";
  local_pose_msg->x = pos_ned.x;          //X in m
  local_pose_msg->y = pos_ned.y;          //Y in m
  local_pose_msg->z = pos_ned.z;          //Z in m

  auto local_velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();
  local_velocity_msg->linear.x = pos_ned.vx;          //Vx in m/s
  local_velocity_msg->linear.y = pos_ned.vy;          //Vy in m/s
  local_velocity_msg->linear.z = pos_ned.vz;          //Vz in m/s

  localPoseActualPublisher_->publish(std::move(local_pose_msg));
  localVelocityActualPublisher_->publish(std::move(local_velocity_msg));
}

//=============================================================================
// Handler for Publishing Global Position and Velocity 
//=============================================================================
void BlueROVBridge::handleGlobalPositionInt(const mavlink_message_t& msg)
{
  mavlink_global_position_int_t pos_int;
  mavlink_msg_global_position_int_decode(&msg, &pos_int);

  auto global_pose_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
  global_pose_msg->header.stamp = this->now();
  global_pose_msg->header.frame_id = "Global WGS84";
  global_pose_msg->x = pos_int.lat ;          //Lat in degE7  
  global_pose_msg->y = pos_int.lon ;          //Lon in degE7
  global_pose_msg->z = -pos_int.alt ;         //Alt in mm

  auto global_velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();
  global_velocity_msg->linear.x = pos_int.vx;  //Vx in cm/s
  global_velocity_msg->linear.y = pos_int.vy;  //Vy in cm/s
  global_velocity_msg->linear.z = pos_int.vz;  //Vz in cm/s

  globalPoseActualPublisher_->publish(std::move(global_pose_msg));  
  globalVelocityActualPublisher_->publish(std::move(global_velocity_msg));
}

//=============================================================================
// Handler for Publishing Attitude and Angular Velocities 
//=============================================================================
void BlueROVBridge::handleAttitude(const mavlink_message_t& msg)
{
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(&msg, &attitude);
  
  // Add a the rotation matrix to the auv_core_helper package
  
  auto local_pose_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
  local_pose_msg->roll = attitude.roll;          //Roll in rad
  local_pose_msg->pitch = attitude.pitch;        //Pitch in rad
  local_pose_msg->yaw = attitude.yaw;            //Yaw in rad

  auto local_velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();
  local_velocity_msg->angular.x = attitude.rollspeed;          //Roll rate in rad/s
  local_velocity_msg->angular.y = attitude.pitchspeed;        //Pitch rate in rad/s
  local_velocity_msg->angular.z = attitude.yawspeed;           //Yaw rate in rad/s
  
  auto global_pose_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
  global_pose_msg->roll = attitude.roll;            //Roll in rad
  global_pose_msg->pitch = attitude.pitch;         //Pitch in rad
  global_pose_msg->yaw = attitude.yaw;             //Yaw in rad

  auto global_velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();
  global_velocity_msg->angular.x = attitude.rollspeed;          //Roll rate in rad/s
  global_velocity_msg->angular.y = attitude.pitchspeed;        //Pitch rate in rad/s
  global_velocity_msg->angular.z = attitude.yawspeed;           //Yaw rate in rad/s
  
  localPoseActualPublisher_->publish(std::move(local_pose_msg));      
  localVelocityActualPublisher_->publish(std::move(local_velocity_msg));
  globalPoseActualPublisher_->publish(std::move(global_pose_msg));
  globalVelocityActualPublisher_->publish(std::move(global_velocity_msg));
}

//=============================================================================
// Handler for Publishing DVL Distance
//=============================================================================
void BlueROVBridge::handleDvlDistance(const mavlink_message_t& msg)
{
  mavlink_distance_sensor_t distance_sensor;
  mavlink_msg_distance_sensor_decode(&msg, &distance_sensor);

  auto dvl_distance_msg = std::make_unique<std_msgs::msg::Float64>();
  dvl_distance_msg->data = distance_sensor.current_distance;  //Distance in cm 

  dvlDistancePublisher_->publish(std::move(dvl_distance_msg));
}

//=============================================================================
// Handler of Command ACK
//=============================================================================
void BlueROVBridge::handleCommandAck(const mavlink_message_t& msg)
{
  // is adding a publisher for the command ack needed?  yes is needed
  
  mavlink_msg_command_ack_decode(&msg, &ack);
  
  // Log ACK messages regardless of the command type
  if (ack.command == MAV_CMD_DO_SET_MODE) {
    if (ack.result == MAV_RESULT_ACCEPTED) {
      RCLCPP_INFO(this->get_logger(), "Mode change accepted by ArduSub");
    } else {
      // Log the specific error code for better diagnostics
      const char* error_str = "Unknown";
      switch (ack.result) {
        case MAV_RESULT_DENIED: error_str = "DENIED"; break;
        case MAV_RESULT_UNSUPPORTED: error_str = "UNSUPPORTED"; break;
        case MAV_RESULT_FAILED: error_str = "FAILED"; break;
        case MAV_RESULT_TEMPORARILY_REJECTED: error_str = "TEMPORARILY_REJECTED"; break;
        default: error_str = "UNKNOWN"; break;
      }
      
      RCLCPP_WARN(this->get_logger(), 
          "Mode change REJECTED by ArduSub (result=%s).", error_str);
    }
  }
}

//=============================================================================
// armingServiceCallback
// Service callback to arm or disarm the vehicle.
//=============================================================================
void BlueROVBridge::armingServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Arming service called: %s", request->data ? "ARM" : "DISARM");
  setArmState(request->data);                                     // True to arm, false to disarm
  response->success = true;                                      
  response->message = request->data ? "Arm command sent." : "Disarm command sent.";
}

//=============================================================================
// flightModeServiceCallback
// Service callback to set the vehicle's flight mode.
//=============================================================================
void BlueROVBridge::flightModeServiceCallback(
    const std::shared_ptr<auv_core_helper::srv::SetFlightMode::Request> request,
    std::shared_ptr<auv_core_helper::srv::SetFlightMode::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Flight mode service called: %s", request->mode.c_str());
  setFlightMode(request->mode);
  response->success = true; // Assuming setFlightMode handles logging/errors
  response->message = "Set flight mode command sent to '" + request->mode + "'.";
}

//=============================================================================
// setArmState
//=============================================================================
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

//=============================================================================
// setFlightMode
// Sets the flight mode without changing arm state
//=============================================================================
void BlueROVBridge::setFlightMode(const std::string& mode)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot set mode yet; no autopilot heartbeat discovered!");
    return;
  }

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
  
  // Send the mode change command
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

    RCLCPP_INFO(this->get_logger(),"Setting flight mode=%s (sys=%d, comp=%d)...",mode.c_str(), target_system_, target_component_);
}

//=============================================================================
// rcChannelValuesDesiredCallback
//=============================================================================
void BlueROVBridge::rcChannelValuesDesiredCallback(const auv_core_helper::msg::RCChannels::SharedPtr msg)
{   
    if (!got_heartbeat_) {
        RCLCPP_WARN(this->get_logger(), 
            "Cannot set RC channel values yet; no autopilot heartbeat discovered!");
        return;
    }
    
    if (hb.custom_mode != MAV_MODE_MANUAL_ARMED) {
        RCLCPP_WARN(this->get_logger(), 
            "Vehicle is not in MANUAL mode. Setting MANUAL mode before sending RC channel values.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "RC channel values desired received");
    RCLCPP_INFO(this->get_logger(), "channels: %d, %d, %d, %d, %d, %d", msg->channels[0], msg->channels[1], msg->channels[2], msg->channels[3], msg->channels[4], msg->channels[5]);
   
  // Initialize all channels .
    uint16_t rc_channel_values[18];
    for (int i = 0; i < 18; ++i) {
        rc_channel_values[i] = UINT16_MAX; //  A value of 0 or UINT16_MAX means to ignore this field
    }

    rc_channel_values[0] = msg->channels[0];
    rc_channel_values[1] = msg->channels[1];
    rc_channel_values[2] = msg->channels[2];
    rc_channel_values[3] = msg->channels[3];
    rc_channel_values[4] = msg->channels[4];
    rc_channel_values[5] = msg->channels[5];

    setRcChannelPwm(rc_channel_values);
}
//=============================================================================
// setRcChannelPwm
//=============================================================================
void BlueROVBridge::setRcChannelPwm(const uint16_t* rc_channel_values)
{
  RCLCPP_INFO(this->get_logger(), "Setting RC channel values");
    // Send MAVLink RC override message
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack(
        system_id_,
        component_id_,
        &msg,
        target_system_,
        target_component_,
        rc_channel_values[0],
        rc_channel_values[1],
        rc_channel_values[2],
        rc_channel_values[3],
        rc_channel_values[4],
        rc_channel_values[5],
        rc_channel_values[6],
        rc_channel_values[7],
        rc_channel_values[8],
        rc_channel_values[9],
        rc_channel_values[10],
        rc_channel_values[11],
        rc_channel_values[12],
        rc_channel_values[13],
        rc_channel_values[14],
        rc_channel_values[15],
        rc_channel_values[16],
        rc_channel_values[17]
    );
    sendMavlinkMessage(msg);
}

//=============================================================================
// localPoseDesiredCallback 
//=============================================================================
void BlueROVBridge::localPoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot set local pose yet; no autopilot heartbeat discovered!");
    return;
  }
// make a topic for feedbacking error status 
// udp with for loop logic 
  if (hb.custom_mode != MAV_MODE_GUIDED_ARMED) {
    RCLCPP_WARN(this->get_logger(), 
        "Vehicle is not in GUIDED mode and Armed. Cannot set local pose.");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Local pose desired received");
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, yaw: %f", position_target_.x, position_target_.y, position_target_.z, position_target_.yaw);

  
  position_target_.time_boot_ms = static_cast<uint32_t>(this->now().nanoseconds() / 1000000);
  position_target_.target_system = target_system_;
  position_target_.target_component = target_component_;
  position_target_.coordinate_frame = MAV_FRAME_LOCAL_NED;
  position_target_.type_mask = MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_POSITION 
  | MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_VELOCITY 
  | MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_YAW 
  | MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_YAW_RATE;

  position_target_.x = msg->x;
  position_target_.y = msg->y;
  position_target_.z = msg->z;
  position_target_.yaw = msg->yaw;

  SetPositionTargetLocalNED(position_target_); 

  //attitude_target_.time_boot_ms = static_cast<uint32_t>(this->now().nanoseconds() / 1000000);
  //attitude_target_.target_system = target_system_;
  //attitude_target_.target_component = target_component_;
  //attitude_target_.type_mask = MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_YAW_ANGLE;
  // Add the a coversion function in the auv_core_helper package from euler angles to quaternion
  //attitude_target_.q[0] = 1.0f;
  //attitude_target_.q[1] = 0.0f;
  //attitude_target_.q[2] = 0.0f;
  //attitude_target_.q[3] = 0.0f;
  //SetAttitudeTarget(attitude_target_);
}
//=============================================================================
// velocityDesiredCallback
//=============================================================================
void BlueROVBridge::velocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Velocity desired received");
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot set velocity yet; no autopilot heartbeat discovered!");
    return;
  }
  
  if (hb.custom_mode != MAV_MODE_GUIDED_ARMED) {
    RCLCPP_WARN(this->get_logger(), 
        "Vehicle is not in GUIDED mode and Armed. Cannot set velocity.");
    return;
  }
  
  position_target_.vx = msg->linear.x;
  position_target_.vy = msg->linear.y;
  position_target_.vz = msg->linear.z;
  position_target_.yaw_rate = msg->angular.z;

  SetPositionTargetLocalNED(position_target_);

  //attitude_target_.body_roll_rate = msg->angular.x;
  //attitude_target_.body_pitch_rate = msg->angular.y;
  //attitude_target_.body_yaw_rate = msg->angular.z;

  //SetAttitudeTarget(attitude_target_);
}

//=============================================================================
// sendLocalWaypointToArdupilot
// Converts a waypoint to MAVLink SET_POSITION_TARGET_LOCAL_NED message
//=============================================================================
/**
 * @brief Send a waypoint to ArduPilot in GUIDED mode
 * 
 * This function converts a ROS waypoint to a MAVLink SET_POSITION_TARGET_LOCAL_NED message and sends it to ArduPilot.
 */
void BlueROVBridge::SetPositionTargetLocalNED(const mavlink_set_position_target_local_ned_t& position_target_)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send waypoint yet; no autopilot heartbeat discovered!");
    return;
  }
  
  mavlink_message_t msg;
  mavlink_msg_set_position_target_local_ned_encode(
      system_id_,
      component_id_,
      &msg,
      &position_target_
  );
  
  sendMavlinkMessage(msg);
  
  RCLCPP_INFO(this->get_logger(), 
      "Sent waypoint to ArduPilot: NED(%.2f, %.2f, %.2f)",
      position_target_.x, position_target_.y, position_target_.z);
}

//=============================================================================
// prepareGlobalPositionTarget
// Prepares a MAVLink SET_POSITION_TARGET_GLOBAL_INT structure
//=============================================================================
void BlueROVBridge::prepareGlobalPositionTarget(int32_t lat_int, int32_t lon_int, float alt, 
                                              mavlink_set_position_target_global_int_t& global_target)
{
  // Clear the structure
  memset(&global_target, 0, sizeof(global_target));
  
  // Set header information
  global_target.time_boot_ms = static_cast<uint32_t>(this->now().nanoseconds() / 1000000);
  global_target.target_system = target_system_;
  global_target.target_component = target_component_;
  
  // Set coordinate frame
  global_target.coordinate_frame = MAV_FRAME_GLOBAL_INT;  // Or MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
  
  // Create type_mask - ignore velocity and acceleration
  // bit set to 1 means "ignore this dimension"
  global_target.type_mask = 0b0000111111000000;  // Ignore velocity, acceleration, yaw, yaw rate
  
  // Set position (lat/lon in degrees*1e7, altitude in meters)
  global_target.lat_int = lat_int;
  global_target.lon_int = lon_int;
  global_target.alt = alt;
  
  // Set velocities and accelerations to zero (not used with the defined type_mask)
  global_target.vx = 0.0f;
  global_target.vy = 0.0f;
  global_target.vz = 0.0f;
  global_target.afx = 0.0f;
  global_target.afy = 0.0f;
  global_target.afz = 0.0f;
  global_target.yaw = 0.0f;
  global_target.yaw_rate = 0.0f;
}

//=============================================================================
// sendGlobalWaypoint
// Sends a waypoint using SET_POSITION_TARGET_GLOBAL_INT message
//=============================================================================
void BlueROVBridge::sendGlobalWaypoint(int32_t lat_int, int32_t lon_int, float alt)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send global waypoint; no autopilot heartbeat discovered!");
    return;
  }
  
  // Ensure we're in GUIDED mode
  if (!guided_mode_active_ && !mode_change_requested_) {
    RCLCPP_WARN(this->get_logger(), 
        "Vehicle not in GUIDED mode. Setting GUIDED mode before sending global waypoint.");
    setFlightMode("GUIDED");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(),
      "Sending global waypoint: lat=%d, lon=%d, alt=%.2f", 
      lat_int, lon_int, alt);
  
  // Create the position target message
  mavlink_set_position_target_global_int_t global_target;
  prepareGlobalPositionTarget(lat_int, lon_int, alt, global_target);
  
  // Create the MAVLink message
  mavlink_message_t msg;
  mavlink_msg_set_position_target_global_int_encode(
      system_id_,
      component_id_,
      &msg,
      &global_target
  );
  
  // Send the message
  sendMavlinkMessage(msg);
  
  RCLCPP_INFO(this->get_logger(), 
      "Global waypoint sent successfully.");
}

//=============================================================================
// sendAttitudeTarget
// Sends MAV_CMD_DO_SET_ATTITUDE_TARGET command to control vehicle attitude
//=============================================================================
//void BlueROVBridge::SetAttitudeTarget(const mavlink_set_attitude_target_t& attitude_target_)
//{
//  if (!got_heartbeat_) {
//    RCLCPP_WARN(this->get_logger(), 
//        "Cannot send ATTITUDE_TARGET command; no autopilot heartbeat discovered!");
//    return;
//  }
  
  // Create the MAVLink message
//  mavlink_message_t msg;
//  mavlink_msg_set_attitude_target_pack(
//      system_id_,
//      component_id_,
//      &msg,
//      target_system_,
//      target_component_,
//      MAVLINK_MSG_ID_SET_ATTITUDE_TARGET,
//      0, // confirmation
//      attitude_target_.type_mask,
//      1, // param1: 0=ignore, 1=take attitude from param2-7
//      0, // param2: roll
//      0, // param3: pitch
//      0, // param4: yaw
//      0, // param5: thrust
//      0, // param6: reserved
//      0, // param7: reserved
//      0, // param8: reserved
//      0, // param9: reserved
//      0, // param10: reserved
//      0, // param11: reserved
//      0  // param12: reserved
//  );
  
//  // Send the message
//  sendMavlinkMessage(msg);
  
//  RCLCPP_INFO(this->get_logger(), "ATTITUDE_TARGET command sent");
//}

//=============================================================================
// sendConditionYaw
// Sends MAV_CMD_CONDITION_YAW command to control vehicle heading
//=============================================================================
void BlueROVBridge::sendConditionYaw(float heading_deg, bool is_relative, int direction, float angular_rate)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send CONDITION_YAW command; no autopilot heartbeat discovered!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(),
      "Setting vehicle heading: %.1f degrees, %s, direction=%d, rate=%.1f deg/s",
      heading_deg, is_relative ? "relative" : "absolute", direction, angular_rate);
  
  // Clamp heading to 0-360 range if absolute
  if (!is_relative) {
    heading_deg = fmod(heading_deg, 360.0f);
    if (heading_deg < 0) {
      heading_deg += 360.0f;
    }
  }
  
  // Create command message
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_CONDITION_YAW,
      0,                               // confirmation
      heading_deg,                     // param1: target angle (degrees)
      angular_rate,                    // param2: angular speed (deg/sec)
      direction,                       // param3: direction: -1=CCW, 1=CW, 0=shortest
      is_relative ? 1.0f : 0.0f,       // param4: 0=absolute, 1=relative
      0.0f, 0.0f, 0.0f                 // param5-7: unused
  );
  
  // Send the message
  sendMavlinkMessage(msg);
  
  RCLCPP_INFO(this->get_logger(), "CONDITION_YAW command sent");
}

//=============================================================================
// sendSetHome
// Sends MAV_CMD_DO_SET_HOME command to set home position
//=============================================================================
void BlueROVBridge::sendSetHome(float latitude, float longitude, float altitude, bool use_current)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send DO_SET_HOME command; no autopilot heartbeat discovered!");
    return;
  }
  
  // Create the position information string properly
  std::string position_str;
  if (use_current) {
    position_str = "current position";
  } else {
    position_str = "lat=" + std::to_string(latitude) + 
                  ", lon=" + std::to_string(longitude) + 
                  ", alt=" + std::to_string(altitude);
  }
  
  RCLCPP_INFO(this->get_logger(), "Setting home position: %s", position_str.c_str());
  
  // Create command message
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_DO_SET_HOME,
      0,                                // confirmation
      use_current ? 1.0f : 0.0f,        // param1: 1=use current position, 0=use specified position
      0.0f,                             // param2: reserved
      0.0f,                             // param3: reserved
      0.0f,                             // param4: yaw angle (not used)
      latitude,                         // param5: latitude (ignored if use_current=1)
      longitude,                        // param6: longitude (ignored if use_current=1)
      altitude                          // param7: altitude (ignored if use_current=1)
  );
  
  // Send the message multiple times for reliability
  constexpr int NUM_RETRIES = 3;
  constexpr int RETRY_DELAY_MS = 100;
  
  for (int i = 0; i < NUM_RETRIES; i++) {
    sendMavlinkMessage(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
  }
  
  RCLCPP_INFO(this->get_logger(), "DO_SET_HOME command sent (repeated %d times)", NUM_RETRIES);
}

//=============================================================================
// sendOverrideGoto
// Sends MAV_CMD_OVERRIDE_GOTO command to interrupt current navigation
//=============================================================================
void BlueROVBridge::sendOverrideGoto(const geometry_msgs::msg::Point& position, bool continue_cmd)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send OVERRIDE_GOTO command; no autopilot heartbeat discovered!");
    return;
  }
  
  if (!guided_mode_active_) {
    RCLCPP_WARN(this->get_logger(), 
        "Vehicle not in GUIDED mode, OVERRIDE_GOTO command may not work as expected");
  }
  
  RCLCPP_INFO(this->get_logger(),
      "Sending MAV_CMD_OVERRIDE_GOTO to position: (%.2f, %.2f, %.2f), continue=%s",
      position.x, position.y, position.z, continue_cmd ? "true" : "false");
  
  // Convert from ENU to NED coordinate system for ArduPilot
  float x_ned = position.y;  // North = East in ENU
  float y_ned = position.x;  // East = North in ENU
  float z_ned = -position.z; // Down = -Up in ENU
  
  // Create command message
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      target_component_,
      MAV_CMD_OVERRIDE_GOTO,
      0,                          // confirmation
      continue_cmd ? 1 : 0,       // param1: 0=Do not continue mission, 1=Continue mission
      MAV_GOTO_DO_HOLD,           // param2: MAV_GOTO enum
      MAV_FRAME_LOCAL_NED,        // param3: Frame (local NED)
      0.0f,                       // param4: Yaw (not used, set to 0)
      x_ned,                      // param5: Latitude/X position
      y_ned,                      // param6: Longitude/Y position
      z_ned                       // param7: Altitude/Z position
  );
  
  // Send the message
  sendMavlinkMessage(msg);
  
  RCLCPP_INFO(this->get_logger(), 
      "OVERRIDE_GOTO command sent to NED position: (%.2f, %.2f, %.2f)",
      x_ned, y_ned, z_ned);
      
  // If this is an emergency command that should interrupt current navigation,
  // update internal state to reflect that
  if (!continue_cmd) {
    waypoint_navigation_active_ = false;
    
    // Create a new waypoint at the override position
    geometry_msgs::msg::PoseStamped override_wp;
    override_wp.header.stamp = this->now();
    override_wp.header.frame_id = "world";
    override_wp.pose.position = position;
    
    // Update the current waypoint
    current_waypoint_ = override_wp;
  }
}