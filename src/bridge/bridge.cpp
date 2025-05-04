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

  // 1) Initialize the MAVLink UDP connection on local port=14551
  initMavlinkConnection();

  // 2) Setup ROS pubs/subs
  localPositionActualPublisher_ = this->create_publisher<auv_core_helper::msg::Position>(auv_core_helper::topicnames::local_position_actual,10);
  globalPositionActualPublisher_ = this->create_publisher<auv_core_helper::msg::Position>(auv_core_helper::topicnames::global_position_actual,10);
  attitudeActualPublisher_ = this->create_publisher<auv_core_helper::msg::Attitude>(auv_core_helper::topicnames::attitude_actual,10);
  dvlDistancePublisher_ = this->create_publisher<std_msgs::msg::Float64>(auv_core_helper::topicnames::dvl_distance_actual,10);

  armedPublisher_ = this->create_publisher<std_msgs::msg::Int8>(auv_core_helper::topicnames::armed,10);
  flightModePublisher_ = this->create_publisher<std_msgs::msg::Int32>(auv_core_helper::topicnames::flight_mode,10);

  poseDesiredSubscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(auv_core_helper::topicnames::pose_desired,10,std::bind(&BlueROVBridge::poseDesiredCallback, this, std::placeholders::_1));
  velocityDesiredSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_desired,10,std::bind(&BlueROVBridge::velocityDesiredCallback, this, std::placeholders::_1));
  accelerationDesiredSubscription_ = this->create_subscription<geometry_msgs::msg::Accel>(auv_core_helper::topicnames::acceleration_desired,10,std::bind(&BlueROVBridge::accelerationDesiredCallback, this, std::placeholders::_1));
  yawRateDesiredSubscription_ = this->create_subscription<std_msgs::msg::Float64>(auv_core_helper::topicnames::yaw_rate_desired,10,std::bind(&BlueROVBridge::yawRateDesiredCallback, this, std::placeholders::_1));
  kclStateSubscription_ = this->create_subscription<std_msgs::msg::String>(auv_core_helper::topicnames::kcl_state,10,std::bind(&BlueROVBridge::kclStateCallback, this, std::placeholders::_1));

  // 3) Timers
  data_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(125), // ~8Hz
      std::bind(&BlueROVBridge::receiveData, this)
  );
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
// Message handler functions
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
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(&msg, &hb);

  // If we haven't yet received the autopilot heartbeat, handle it:
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

    // Configure data streams directly (merged from requestDataStreams)
    setMessageInterval(MAVLINK_MSG_ID_ATTITUDE, 8.0f); // #30
    setMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 8.0f); // #32

    RCLCPP_INFO(this->get_logger(),
        "Configured message intervals for ATTITUDE(#30) and LOCAL_POSITION_NED(#32) at 8 Hz.");
  }
}

//=============================================================================
// Postion and Velocity in NED
//=============================================================================
void BlueROVBridge::handleLocalPositionNed(const mavlink_message_t& msg)
{
  mavlink_local_position_ned_t pos_ned;
  mavlink_msg_local_position_ned_decode(&msg, &pos_ned);

  auto local_position_msg = std::make_unique<auv_core_helper::msg::Position>();
  local_position_msg->timestamp = pos_ned.time_boot_ms;
  local_position_msg->x = pos_ned.x;
  local_position_msg->y = pos_ned.y;
  local_position_msg->z = pos_ned.z;
  local_position_msg->vx = pos_ned.vx;
  local_position_msg->vy = pos_ned.vy;
  local_position_msg->vz = pos_ned.vz;

  localPositionActualPublisher_->publish(std::move(local_position_msg));

  if (!home_pose_set_) {
    poseHome_[0] = poseActual_[0];
    poseHome_[1] = poseActual_[1];
    poseHome_[2] = poseActual_[2];
    home_pose_set_ = true;
  }
}

//=============================================================================
// Global Position
//=============================================================================
void BlueROVBridge::handleGlobalPositionInt(const mavlink_message_t& msg)
{
  mavlink_global_position_int_t pos_int;
  mavlink_msg_global_position_int_decode(&msg, &pos_int);

  auto global_position_msg = std::make_unique<auv_core_helper::msg::Position>();
  global_position_msg->timestamp = pos_int.time_boot_ms;
  global_position_msg->x = pos_int.lat / 1.0e7;
  global_position_msg->y = pos_int.lon / 1.0e7;
  global_position_msg->z = -pos_int.alt / 1.0e3;
  global_position_msg->vx = pos_int.vx;
  global_position_msg->vy = pos_int.vy;
  global_position_msg->vz = pos_int.vz;

  globalPositionActualPublisher_->publish(std::move(global_position_msg));
}

//=============================================================================
// Attitude
//=============================================================================
void BlueROVBridge::handleAttitude(const mavlink_message_t& msg)
{
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(&msg, &attitude);

  auto attitude_msg = std::make_unique<auv_core_helper::msg::Attitude>();
  attitude_msg->timestamp = attitude.time_boot_ms;
  attitude_msg->roll = attitude.roll;
  attitude_msg->pitch = attitude.pitch;
  attitude_msg->yaw = attitude.yaw;
  attitude_msg->rollrate = attitude.rollspeed;
  attitude_msg->pitchrate = attitude.pitchspeed;
  attitude_msg->yawrate = attitude.yawspeed;
  
  if (!home_pose_set_) {
    poseHome_[3] = poseActual_[3];
    poseHome_[4] = poseActual_[4];
    poseHome_[5] = poseActual_[5];
  }
  
  attitudeActualPublisher_->publish(*attitude_msg);
}

//=============================================================================
// DVL Distance
//=============================================================================
void BlueROVBridge::handleDvlDistance(const mavlink_message_t& msg)
{
  mavlink_distance_sensor_t distance_sensor;
  mavlink_msg_distance_sensor_decode(&msg, &distance_sensor);

  auto dvl_distance_msg = std::make_unique<std_msgs::msg::Float64>();
  dvl_distance_msg->data = distance_sensor.current_distance;

  dvlDistancePublisher_->publish(*dvl_distance_msg);
}

//=============================================================================
// Command ACK
//=============================================================================
void BlueROVBridge::handleCommandAck(const mavlink_message_t& msg)
{
  mavlink_command_ack_t ack;
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
  } else {
    // Only log non-mode change ACKs at DEBUG level to reduce noise
    RCLCPP_DEBUG(this->get_logger(),
        "CMD_ACK received: command=%u, result=%u",
        ack.command, ack.result);
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
  // param1 = message_id
  // param2 = desired interval in microseconds
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
// velocityDesiredCallback
//=============================================================================
void BlueROVBridge::velocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Store the desired velocity
  velocityDesired_[0] = msg->linear.x;
  velocityDesired_[1] = msg->linear.y;
  velocityDesired_[2] = msg->linear.z;
  velocityDesired_[3] = msg->angular.x;
  velocityDesired_[4] = msg->angular.y;
  velocityDesired_[5] = msg->angular.z;
}

//=============================================================================
// kclStateCallback
//   - If state="IDLE": disarm
//   - If state="ARM": arm without changing mode
//   - If state="MANUAL", "STABILIZE", etc.: set flight mode (possibly while staying armed)
//   - If "PATH_FOLLOWING": reset home pose
//   - If "WAYPOINT_NAVIGATION": set GUIDED mode for waypoint navigation
//=============================================================================
void BlueROVBridge::kclStateCallback(const std_msgs::msg::String::SharedPtr msg)
{  
  if (msg->data == "IDLE")
    setArmState(false);
  else if(msg->data == "ARM")
    setArmState(true);
  else
    setFlightMode(msg->data);
}

//=============================================================================
// setArmState
// Arms or disarms the vehicle and optionally sets the mode after arming.
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

  RCLCPP_INFO(this->get_logger(),
      "Setting flight mode=%s (sys=%d, comp=%d)...",
      mode.c_str(), target_system_, target_component_);

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
  
  // Send multiple times for reliability (UDP is unreliable)
  constexpr int NUM_RETRIES = 3;
  constexpr int RETRY_DELAY_MS = 20;
  
  for (int i = 0; i < NUM_RETRIES; i++) {
    sendMavlinkMessage(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
  }
  
  RCLCPP_INFO(this->get_logger(), 
      "Set mode command sent (%s, sent %d times).", mode.c_str(), NUM_RETRIES);
      
  // Set the requested mode and track the request time
  // Note: we don't immediately set guided_mode_active_/position_hold_active_ flags
  // Instead, we wait for confirmation via heartbeat
  requested_mode_ = mode;
  mode_change_requested_ = true;
  mode_change_request_time_ = this->now();
  mode_change_attempts_++;
  
  // Log the mode change attempt number
  if (mode_change_attempts_ > 1) {
    RCLCPP_INFO(this->get_logger(), "Mode change attempt #%d", mode_change_attempts_);
  }
}

//=============================================================================
// setRcChannelPwm
//=============================================================================
void BlueROVBridge::setRcChannelPwm(const Eigen::VectorXd& velocityDesiredPwm)
{
    // Check the size of the input vector
    if (velocityDesiredPwm.size() != 6) {
        RCLCPP_ERROR(this->get_logger(),
                     "setRcChannelPwm: velocityDesiredPwm must have exactly 6 elements.");
        return;
    }

    // Initialize all channels to "do not change" (65535).
    uint16_t rc_channel_values[18];
    for (int i = 0; i < 18; ++i) {
        rc_channel_values[i] = 65535; // Do not override these channels
    }

    // Map velocityDesiredPwm to the correct channel order:
    //
    // Channel 1 -> pitch    = velocityDesiredPwm(4)
    // Channel 2 -> roll     = velocityDesiredPwm(3)
    // Channel 3 -> vertical = velocityDesiredPwm(2)
    // Channel 4 -> yaw      = velocityDesiredPwm(5)
    // Channel 5 -> forward  = velocityDesiredPwm(0)
    // Channel 6 -> lateral  = velocityDesiredPwm(1)
    rc_channel_values[0] = static_cast<uint16_t>(velocityDesiredPwm(4));  // pitch
    rc_channel_values[1] = static_cast<uint16_t>(velocityDesiredPwm(3));  // roll
    rc_channel_values[2] = static_cast<uint16_t>(velocityDesiredPwm(2));  // vertical
    rc_channel_values[3] = static_cast<uint16_t>(velocityDesiredPwm(5));  // yaw
    rc_channel_values[4] = static_cast<uint16_t>(velocityDesiredPwm(0));  // forward
    rc_channel_values[5] = static_cast<uint16_t>(velocityDesiredPwm(1));  // lateral

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
// sendWaypointToArdupilot
// Converts a waypoint to MAVLink SET_POSITION_TARGET_LOCAL_NED message
//=============================================================================
/**
 * @brief Send a waypoint to ArduPilot in GUIDED mode
 * 
 * This function converts a ROS waypoint (in ENU coordinates) to a MAVLink
 * SET_POSITION_TARGET_LOCAL_NED message (in NED coordinates) and sends it
 * to ArduPilot.
 * 
 * @param waypoint The waypoint position in ENU coordinates
 */
void BlueROVBridge::sendWaypointToArdupilot(const geometry_msgs::msg::PoseStamped& waypoint)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot send waypoint yet; no autopilot heartbeat discovered!");
    return;
  }
  
  // Convert waypoint from ENU to NED coordinate system for ArduPilot
  mavlink_set_position_target_local_ned_t position_target;
  convertENUtoNED(waypoint, position_target);
  
  // Create the MAVLink message
  mavlink_message_t msg;
  mavlink_msg_set_position_target_local_ned_encode(
      system_id_,
      component_id_,
      &msg,
      &position_target
  );
  
  // Send the message
  sendMavlinkMessage(msg);
  
  RCLCPP_INFO(this->get_logger(), 
      "Sent waypoint to ArduPilot: NED(%.2f, %.2f, %.2f)",
      position_target.x, position_target.y, position_target.z);
      
  // Calculate desired yaw to point towards the waypoint
  if (home_pose_set_) {
    // Calculate the current position in NED
    double relative_x_ned = poseActual_[0] - poseHome_[0];
    double relative_y_ned = poseActual_[1] - poseHome_[1];
    
    // Calculate vector to target from current position
    double dx = position_target.x - relative_x_ned;
    double dy = position_target.y - relative_y_ned;
    
    // Calculate heading in degrees (0 is North, positive clockwise)
    double heading_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    
    // Convert to 0-360 range
    if (heading_deg < 0) {
      heading_deg += 360.0;
    }
    
    // Send yaw command to face the waypoint
    // 0=shortest path determination, 5=reasonable rotation rate (deg/sec)
    sendConditionYaw(static_cast<float>(heading_deg), false, 0, 5.0f);
    
    RCLCPP_INFO(this->get_logger(), 
        "Setting heading towards waypoint: %.1f degrees", heading_deg);
  }
}

//=============================================================================
// convertENUtoNED
// Converts waypoint from ENU coordinates to NED coordinates for ArduPilot
//=============================================================================
/**
 * @brief Convert waypoint from ENU to NED coordinate system
 * 
 * Coordinate conversion from ROS (ENU) to ArduPilot (NED):
 * NED.x = ENU.y  (North = East)
 * NED.y = ENU.x  (East = North) 
 * NED.z = -ENU.z (Down = -Up)
 * 
 * @param enu Input waypoint in ENU coordinates
 * @param ned Output structure in NED coordinates for MAVLink
 */
void BlueROVBridge::convertENUtoNED(const geometry_msgs::msg::PoseStamped& enu, 
                                   mavlink_set_position_target_local_ned_t& ned)
{
  // Clear the structure
  memset(&ned, 0, sizeof(ned));
  
  // Set header information
  ned.time_boot_ms = static_cast<uint32_t>(this->now().nanoseconds() / 1000000);
  ned.target_system = target_system_;
  ned.target_component = target_component_;
  ned.coordinate_frame = MAV_FRAME_LOCAL_NED;
  
  // Create mask 
  // Each bit set to 1 means "ignore this dimension"
  ned.type_mask =0b0000000111000000 ; 
  
  
  // Convert ENU to NED coordinates
  ned.x = enu.pose.position.y;  // North = East (ENU.y -> NED.x)
  ned.y = enu.pose.position.x;  // East = North (ENU.x -> NED.y)
  ned.z = -enu.pose.position.z; // Down = -Up (negative ENU.z -> NED.z)
  
  // Set velocities and accelerations to zero (not used with the defined type_mask)
  ned.vx = 0.0f;
  ned.vy = 0.0f;
  ned.vz = 0.0f;
  ned.afx = 0.0f;
  ned.afy = 0.0f;
  ned.afz = 0.0f;
  ned.yaw = 0.0f;
  ned.yaw_rate = 0.0f;
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
    
    // Clear the waypoint queue (optional, depending on desired behavior)
    std::queue<geometry_msgs::msg::PoseStamped> empty;
    std::swap(waypoint_queue_, empty);
    
    // Create a new waypoint at the override position
    geometry_msgs::msg::PoseStamped override_wp;
    override_wp.header.stamp = this->now();
    override_wp.header.frame_id = "world";
    override_wp.pose.position = position;
    
    // Update the current waypoint
    current_waypoint_ = override_wp;
  }
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
  
  // If we're using the current position, update our internal home pose
  if (use_current) {
    poseHome_ = poseActual_;
    home_pose_set_ = true;
    
    RCLCPP_INFO(this->get_logger(),
        "Updated internal home pose to current position: NED=(%.2f, %.2f, %.2f)",
        poseHome_[0], poseHome_[1], poseHome_[2]);
  }
}

//=============================================================================
// STUB: poseDesiredCallback - TODO: Implement actual logic
//=============================================================================
void BlueROVBridge::poseDesiredCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_WARN(this->get_logger(), "poseDesiredCallback received data but is not implemented!");
  // Suppress unused parameter warning
  (void)msg;
}

//=============================================================================
// STUB: accelerationDesiredCallback - TODO: Implement actual logic
//=============================================================================
void BlueROVBridge::accelerationDesiredCallback(const geometry_msgs::msg::Accel::SharedPtr msg)
{
  RCLCPP_WARN(this->get_logger(), "accelerationDesiredCallback received data but is not implemented!");
  // Suppress unused parameter warning
  (void)msg;
}

//=============================================================================
// STUB: yawRateDesiredCallback - TODO: Implement actual logic
//=============================================================================
void BlueROVBridge::yawRateDesiredCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  RCLCPP_WARN(this->get_logger(), "yawRateDesiredCallback received data (%.2f deg/s) but is not implemented!", msg->data);
  // Suppress unused parameter warning
  (void)msg;
  // Example: Might use MAV_CMD_CONDITION_YAW with rate or SET_ATTITUDE_TARGET
}