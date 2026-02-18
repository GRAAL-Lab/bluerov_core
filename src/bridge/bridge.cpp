#include "bluerov-bridge/bluerov_bridge.hpp"
#include "auv_core_helper/helper_lib.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// C / C++ Includes
#include <chrono>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <array>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

namespace {

constexpr double kGravityMps2 = 9.80550;
constexpr double kMilligToMps2 = kGravityMps2 / 1000.0;
constexpr double kMradpsToRadps = 1.0 / 1000.0;
constexpr uint64_t kEpochUsecThreshold = 1000000000000ULL;
constexpr double kNominalFallbackVoltageV = 16.0;

std::string trim(const std::string& input) {
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

bool parseDoubleField(const std::string& token, double* value) {
  const std::string trimmed = trim(token);
  if (trimmed.empty()) {
    return false;
  }

  size_t parsed_chars = 0;
  try {
    *value = std::stod(trimmed, &parsed_chars);
  } catch (const std::exception&) {
    return false;
  }

  return parsed_chars == trimmed.size();
}

std::string getDefaultThrustCsvPath() {
  static const std::array<std::string, 3> kCandidatePaths = {{
      "/home/attia/bluerov_ws/src/bluerov-bridge/docs/T200_PWM_N.csv",
      "/home/attia/bluerov_ws/src/bridge/docs/T200_PWM_N.csv",
      "src/bridge/docs/T200_PWM_N.csv"}};

  try {
    const std::string package_share = ament_index_cpp::get_package_share_directory("bridge");
    const std::string packaged_path = package_share + "/docs/T200_PWM_N.csv";
    if (std::filesystem::exists(packaged_path)) {
      return packaged_path;
    }
  } catch (const std::exception&) {
    // Fall back to workspace paths below.
  }

  for (const auto& candidate_path : kCandidatePaths) {
    if (std::filesystem::exists(candidate_path)) {
      return candidate_path;
    }
  }

  return kCandidatePaths[1];
}

double extractBatteryVoltageV(const mavlink_battery_status_t& battery_status) {
  double voltage_sum_v = 0.0;
  size_t valid_cells = 0;
  for (size_t i = 0; i < 10; ++i) {
    const uint16_t mv = battery_status.voltages[i];
    if (mv == UINT16_MAX || mv == 0) {
      continue;
    }
    voltage_sum_v += static_cast<double>(mv) / 1000.0;
    ++valid_cells;
  }

  if (valid_cells == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return voltage_sum_v;
}

}  // namespace

const rclcpp::Duration BlueROVBridge::kSrvTimeout =rclcpp::Duration::from_seconds(3.0);
const rclcpp::Duration BlueROVBridge::HeartbeatTimeout = rclcpp::Duration::from_seconds(4.0);

/**
 * @file bluerov_bridge.cpp
 * @brief Implementation of the BlueROVBridge class that connects ROS2 to ArduSub via MAVLink
 * Communication is via MAVLink over UDP, using the standard ArduSub protocol.
 */
BlueROVBridge::BlueROVBridge(const rclcpp::NodeOptions& options): Node("mavlink_bridge", options){
  // Declare and get config_name parameter
  this->declare_parameter<std::string>("config_name", "bridge");
  this->declare_parameter<std::string>("imu_topic", imu_topic_);
  this->declare_parameter<std::string>("imu_frame_id", imu_frame_id_);
  this->declare_parameter<std::string>("forces_desired_topic", forces_desired_topic_);
  this->declare_parameter<std::string>("forces_actual_topic", forces_actual_topic_);
  this->declare_parameter<std::string>("pressure_topic", pressure_topic_);
  this->declare_parameter<std::string>("pressure_frame_id", pressure_frame_id_);
  this->declare_parameter<std::string>("thrust_table_csv_path", getDefaultThrustCsvPath());
  std::string configNameParam;
  this->get_parameter("config_name", configNameParam);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("imu_frame_id", imu_frame_id_);
  this->get_parameter("forces_desired_topic", forces_desired_topic_);
  this->get_parameter("forces_actual_topic", forces_actual_topic_);
  this->get_parameter("pressure_topic", pressure_topic_);
  this->get_parameter("pressure_frame_id", pressure_frame_id_);
  this->get_parameter("thrust_table_csv_path", thrust_table_csv_path_);

  LoadBridgeParamsFromConf(
      configNameParam,
      &simulation_mode_,
      &remote_addr_str_,
      reinterpret_cast<int*>(&system_id_),
      reinterpret_cast<int*>(&component_id_),
      &port_);

  RCLCPP_INFO(this->get_logger(), "Starting BlueROVBridge node (UDP port %d, remote_addr: %s, sysid: %d, compid: %d)",
              port_, remote_addr_str_.c_str(), system_id_, component_id_);

  if (!loadThrustTableFromCsv(thrust_table_csv_path_)) {
    throw std::runtime_error("Failed loading thrust table CSV from: " + thrust_table_csv_path_);
  }

  // Initialize the MAVLink UDP connection on local port
  initMavlinkConnection();

  // Setup ROS pubs/subs
  ardusubHeartBeatPublisher_ = this->create_publisher<auv_core_helper::msg::HeartBeat>(auv_core_helper::topicnames::ardusub_heartbeat,1);
  bridgeHeartBeatPublisher_ = this->create_publisher<std_msgs::msg::Int8>(auv_core_helper::topicnames::bridge_heartbeat,1);
  globalOriginPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::global_origin,1);
  batteryStatusPublisher_ = this->create_publisher<auv_core_helper::msg::BatteryStatus>(auv_core_helper::topicnames::battery_status,1);
  globalPoseActualPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_actual_global_,1);
  globalVelocityActualPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_actual_global,1);
  dvlDistancePublisher_ = this->create_publisher<std_msgs::msg::Float64>(auv_core_helper::topicnames::dvl_distance_actual,1);
  ekfStatusPublisher_ = this->create_publisher<std_msgs::msg::Int32>(auv_core_helper::topicnames::ekf_status,1);
  gimbalStatusPublisher_ = this->create_publisher<auv_core_helper::msg::GimbalStatus>(auv_core_helper::topicnames::gimbal_attitude_status,1);
  imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 20);
  forcesDesiredPublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(forces_desired_topic_, 10);
  forcesActualPublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(forces_actual_topic_, 10);
  pressureScaled2Publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(pressure_topic_, 20);

  forces_actual_msg_.name = {
      "thruster_1",
      "thruster_2",
      "thruster_3",
      "thruster_4",
      "thruster_5",
      "thruster_6",
      "thruster_7",
      "thruster_8"};
  forces_actual_msg_.effort.assign(kThrusterCount, 0.0);

  safetySwitchSubscription_ = this->create_subscription<std_msgs::msg::Bool>(auv_core_helper::topicnames::safety_switch,10,std::bind(&BlueROVBridge::safetySwitchCallback, this, std::placeholders::_1));
  globalPoseDesiredSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_desired_global,10,std::bind(&BlueROVBridge::globalPoseDesiredCallback, this, std::placeholders::_1));
  globalVelocityDesiredSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_desired_global,10,std::bind(&BlueROVBridge::globalVelocityDesiredCallback, this, std::placeholders::_1));
  desiredCtrlModeSubscription_ = this->create_subscription<std_msgs::msg::String>(auv_core_helper::topicnames::desired_ctrl_mode, 10, std::bind(&BlueROVBridge::desiredCtrlModeCallback, this, std::placeholders::_1));

  setGlobalOriginService_ = this->create_service<auv_core_helper::srv::SetGlobalOrigin>(auv_core_helper::topicnames::set_global_origin_service, std::bind(&BlueROVBridge::setGlobalOriginServiceCallback, this,std::placeholders::_1, std::placeholders::_2));
  armingService_ = this->create_service<SetBoolSrv>(auv_core_helper::topicnames::arming_service,std::bind(&BlueROVBridge::armingServiceCallback,this, std::placeholders::_1, std::placeholders::_2));
  flightModeService_ = this->create_service<SetModeSrv>(auv_core_helper::topicnames::flight_mode_service,std::bind(&BlueROVBridge::flightModeServiceCallback,this, std::placeholders::_1, std::placeholders::_2));
  gimbalService_ = this->create_service<auv_core_helper::srv::SetGimbalAttitude>(auv_core_helper::topicnames::gimbal_service,std::bind(&BlueROVBridge::gimbalServiceCallback,this, std::placeholders::_1, std::placeholders::_2));

  // Timers
  bridge_heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BlueROVBridge::bridgeHeartbeat, this)); // ~1Hz
  ardusub_heartbeat_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BlueROVBridge::ardusubHeartbeatWatchdog, this)); // ~1Hz
  data_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&BlueROVBridge::receiveData, this)); // ~100Hz
  exec_timer_ = this->create_wall_timer(std::chrono::milliseconds(33),std::bind(&BlueROVBridge::Execute, this)); // ~30Hz

  // Initialize the goal variables
  poseGoalGlobal.setZero();
  poseGoalGlobalLast.setConstant(std::numeric_limits<double>::quiet_NaN());
  velGoalGlobalLast.setConstant(std::numeric_limits<double>::quiet_NaN());
  velocityGoalGlobal.setZero();

  last_heartbeat_time_ = this->now();
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
 * 1. Creates a UDP socket bound to port (standard ArduSub port)
 * 2. Sets up initial remote_addr_ 
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
  local_addr.sin_port        = htons(port_);  // Match BlueOS MAVLink endpoint

  // Bind the socket
  if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
    const std::string err_msg = "Socket bind failed on port " + std::to_string(port_) + ": " + std::string(strerror(errno));
    RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
    close(sock_fd_);
    throw std::runtime_error(err_msg);
  }

  RCLCPP_INFO(this->get_logger(),
      "Bound to %s:%d ", remote_addr_str_.c_str(), port_);

  // Initialize remote address (will be updated when first heartbeat is received)
  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = inet_addr(remote_addr_str_.c_str()); 
  remote_addr_.sin_port = htons(port_); 

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
            handleArduSubHeartbeat(msg, sender_addr);
            break;
            
          case MAVLINK_MSG_ID_ATTITUDE:
            handleAttitude(msg);
            break;

          case MAVLINK_MSG_ID_RAW_IMU:
            handleRawImu(msg);
            break;

          case MAVLINK_MSG_ID_SCALED_PRESSURE2:
            handleScaledPressure2(msg);
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

          case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            handleServoOutputRaw(msg);
            break;

          case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
            handleGlobalOrigin(msg);
            break;

          case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
            handleEkfStatus(msg);
            break;

          case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:  
            handleGimbalStatus(msg);
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

void BlueROVBridge::bridgeHeartbeat() {

  std_msgs::msg::Int8 bridgeHeartbeatMsg;
  bridgeHeartbeatMsg.data = 1;
  bridgeHeartBeatPublisher_->publish(bridgeHeartbeatMsg);

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
      system_id_,
      component_id_,
      &msg,
      MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED,
      0,
      MAV_STATE_ACTIVE);

  sendMavlinkMessage(msg);
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
  
  const char* message_name = get_message_name(message_id);
  RCLCPP_INFO(this->get_logger(),
      "Requested message #%d '%s' at %.1f Hz (%.0f us).",
      message_id, message_name, frequency_hz, interval_us);
}

const char* BlueROVBridge::get_message_name(uint16_t message_id) {
  switch (message_id) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return "HEARTBEAT";
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      return "GLOBAL_POSITION_INT";
    case MAVLINK_MSG_ID_ATTITUDE:
      return "ATTITUDE";
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      return "BATTERY_STATUS";
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
      return "SERVO_OUTPUT_RAW";
    case MAVLINK_MSG_ID_RAW_IMU:
      return "RAW_IMU";
    case MAVLINK_MSG_ID_SCALED_PRESSURE2:
      return "SCALED_PRESSURE2";
    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
      return "GPS_GLOBAL_ORIGIN";
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
      return "DISTANCE_SENSOR";
    case MAVLINK_MSG_ID_COMMAND_ACK:
      return "COMMAND_ACK";  
    case MAVLINK_MSG_ID_ESTIMATOR_STATUS:  
      return "ESTIMATOR_STATUS";
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:  
      return "GIMBAL_DEVICE_ATTITUDE_STATUS";  
    default:
      return "Unknown";
  }
}

void BlueROVBridge::handleArduSubHeartbeat(const mavlink_message_t& msg, const sockaddr_in& sender_addr){

  mavlink_msg_heartbeat_decode(&msg, &hb);

  // If the message is not from an autopilot, ignore it
  if (hb.type == MAV_AUTOPILOT_INVALID) return;

  auto heartBeatMsg = std::make_unique<auv_core_helper::msg::HeartBeat>();
  heartBeatMsg->type = hb.type;
  heartBeatMsg->base_mode = hb.base_mode;
  heartBeatMsg->custom_mode = hb.custom_mode;
  heartBeatMsg->system_status = hb.system_status;

  ardusubHeartBeatPublisher_->publish(*heartBeatMsg);

  if (!got_heartbeat_ && (hb.type == MAV_TYPE_SUBMARINE) ) {
    target_system_    = msg.sysid;
    target_component_ = msg.compid;
    got_heartbeat_    = true;

    last_heartbeat_time_ = this->now();

    // Overwrite remote_addr_ with the sender's IP:port for simulation mode
    if (simulation_mode_){
      remote_addr_ = sender_addr;
      RCLCPP_INFO(this->get_logger(),
          "In simulation mode, overriding remote address with sender's IP:port %s:%d",
          inet_ntoa(sender_addr.sin_addr), ntohs(sender_addr.sin_port));
    }

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(sender_addr.sin_addr), ip_str, sizeof(ip_str));
    uint16_t sender_port = ntohs(sender_addr.sin_port);

    RCLCPP_INFO(this->get_logger(),
        "Got AUTOPILOT heartbeat from target sys=%d, target comp=%d at %s:%d ",
        target_system_, target_component_, ip_str, sender_port);

    // Configure data streams directly 
    setMessageInterval(MAVLINK_MSG_ID_HEARTBEAT, 1.0f);              // #0
    setMessageInterval(MAVLINK_MSG_ID_RAW_IMU, 100.0f); // #27
    setMessageInterval(MAVLINK_MSG_ID_SCALED_PRESSURE2, 100.0f); // #137
    setMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0f);   // #33
    setMessageInterval(MAVLINK_MSG_ID_DISTANCE_SENSOR, 5.0f);        // #34
    setMessageInterval(MAVLINK_MSG_ID_COMMAND_ACK, 8.0f);            // #35
    setMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, 2.0f);         // #147
    setMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20.0f);      // #36
    setMessageInterval(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 1.0f);      // #32
    setMessageInterval(MAVLINK_MSG_ID_ESTIMATOR_STATUS, 5.0f);       // #278
    setMessageInterval(MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, 5.0f); // #285
  } 
  else if (msg.sysid == target_system_ && msg.compid == target_component_) {
    last_heartbeat_time_ = this->now();
    return;
  }
}

void BlueROVBridge::ardusubHeartbeatWatchdog() {
  
  if (!got_heartbeat_) return;

  rclcpp::Duration elapsed = this->now() - last_heartbeat_time_;

  if (elapsed > HeartbeatTimeout) {
    RCLCPP_WARN(get_logger(), "Heartbeat timeout—autopilot disconnected/rebooted");
    got_heartbeat_ = false;
    target_system_ = 0;
    target_component_ = 0;
    last_heartbeat_time_ = rclcpp::Time(0,0,this->get_clock()->get_clock_type());
  }
}

void BlueROVBridge::handleCommandAck(const mavlink_message_t& msg)
{
  mavlink_command_ack_t ack;
  mavlink_msg_command_ack_decode(&msg, &ack);

  auto ack_failed = (ack.result == MAV_RESULT_DENIED ||
                     ack.result == MAV_RESULT_FAILED ||
                     ack.result == MAV_RESULT_TEMPORARILY_REJECTED);

  if (pending_arm_ && ack.command == MAV_CMD_COMPONENT_ARM_DISARM && ack.result == MAV_RESULT_ACCEPTED) {
      bool armed_flag = pending_arm_->want_arm;
      pending_arm_->resp->success = true;
      pending_arm_->resp->message = armed_flag ? "Vehicle armed." : "Vehicle disarmed.";
      armingService_->send_response(*pending_arm_->header, *pending_arm_->resp);
      pending_arm_.reset();
  } else if (pending_arm_ && ack.command == MAV_CMD_COMPONENT_ARM_DISARM && ack_failed) {
    pending_arm_->resp->success = false;
    pending_arm_->resp->message = "Autopilot rejected arming/disarming.";
    armingService_->send_response(*pending_arm_->header, *pending_arm_->resp);
    pending_arm_.reset();
  }

  if (pending_mode_ && ack.command == MAV_CMD_DO_SET_MODE && ack.result == MAV_RESULT_ACCEPTED) {
      pending_mode_->resp->success = true;
      pending_mode_->resp->message = "Flight mode engaged.";
      flightModeService_->send_response(*pending_mode_->header, *pending_mode_->resp);
      pending_mode_.reset();
  } else  if (pending_mode_ && ack.command == MAV_CMD_DO_SET_MODE && ack_failed) {
    pending_mode_->resp->success = false;
    pending_mode_->resp->message = "Autopilot rejected flight-mode change.";
    flightModeService_->send_response(*pending_mode_->header, *pending_mode_->resp);
    pending_mode_.reset();
  } 

  if (pending_gimbal_attitude_ && ack.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW && ack.result == MAV_RESULT_ACCEPTED) {
      pending_gimbal_attitude_->resp->success = true;
      pending_gimbal_attitude_->resp->message = "Gimbal attitude set.";
      gimbalService_->send_response(*pending_gimbal_attitude_->header, *pending_gimbal_attitude_->resp);
      pending_gimbal_attitude_.reset();
  } else  if (pending_gimbal_attitude_ && ack.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW && ack_failed) {
    pending_gimbal_attitude_->resp->success = false;
    pending_gimbal_attitude_->resp->message = "Autopilot rejected gimbal attitude change.";
    gimbalService_->send_response(*pending_gimbal_attitude_->header, *pending_gimbal_attitude_->resp);
    pending_gimbal_attitude_.reset();
  } 
}

void BlueROVBridge::handleGlobalOrigin(const mavlink_message_t& msg){
 
  mavlink_msg_gps_global_origin_decode(&msg, &gps_global_origin);
  
  auto global_origin_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
  global_origin_msg->header.stamp = this->now();
  global_origin_msg->header.frame_id = "Global WGS84";
  global_origin_msg->position.latitude = gps_global_origin.latitude / 1e7;  // Convert to degrees 
  global_origin_msg->position.longitude = gps_global_origin.longitude / 1e7;  // Convert to degrees
  global_origin_msg->depth = gps_global_origin.altitude / 1000.0;  // mm → meters
  globalOriginPublisher_->publish(*global_origin_msg);
}

void BlueROVBridge::handleEkfStatus(const mavlink_message_t& msg){
  mavlink_estimator_status_t ekf_status_report;
  mavlink_msg_estimator_status_decode(&msg, &ekf_status_report);

  auto ekf_status_report_msg = std::make_unique<std_msgs::msg::Int32>();
  ekf_status_report_msg->data = ekf_status_report.flags;

  ekfStatusPublisher_->publish(*ekf_status_report_msg);
}

 void BlueROVBridge::handleDvlDistance(const mavlink_message_t& msg){
  mavlink_distance_sensor_t distance_sensor;
  mavlink_msg_distance_sensor_decode(&msg, &distance_sensor);

  auto dvl_distance_msg = std::make_unique<std_msgs::msg::Float64>();
  dvl_distance_msg->data = distance_sensor.current_distance;  //Distance in cm 

  dvlDistancePublisher_->publish(std::move(dvl_distance_msg));
}

void BlueROVBridge::handleBatteryStatus(const mavlink_message_t& msg){
   mavlink_battery_status_t battery_status;
   mavlink_msg_battery_status_decode(&msg, &battery_status);

  latest_battery_voltage_v_ = extractBatteryVoltageV(battery_status);

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

bool BlueROVBridge::loadThrustTableFromCsv(const std::string& csv_path) {
  thrust_pwm_values_.clear();
  thrust_table_newtons_.clear();
  thrust_table_loaded_ = false;

  std::ifstream csv_file(csv_path);
  if (!csv_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open thrust table CSV: %s", csv_path.c_str());
    return false;
  }

  std::string line;
  size_t line_number = 0;
  while (std::getline(csv_file, line)) {
    ++line_number;
    if (trim(line).empty()) {
      continue;
    }

    std::stringstream line_stream(line);
    std::array<std::string, 4> tokens{};
    size_t token_count = 0;
    while (token_count < tokens.size() && std::getline(line_stream, tokens[token_count], ',')) {
      ++token_count;
    }

    if (token_count < 4) {
      if (line_number == 1) {
        continue;
      }
      RCLCPP_WARN(this->get_logger(), "Skipping malformed thrust CSV line %zu: '%s'", line_number, line.c_str());
      continue;
    }

    double pwm_us = 0.0;
    double force_14v_n = 0.0;
    double force_16v_n = 0.0;
    double force_18v_n = 0.0;

    const bool valid_row =
        parseDoubleField(tokens[0], &pwm_us) &&
        parseDoubleField(tokens[1], &force_14v_n) &&
        parseDoubleField(tokens[2], &force_16v_n) &&
        parseDoubleField(tokens[3], &force_18v_n);

    if (!valid_row) {
      if (line_number == 1) {
        continue;
      }
      RCLCPP_WARN(this->get_logger(), "Skipping non-numeric thrust CSV line %zu: '%s'", line_number, line.c_str());
      continue;
    }

    if (!thrust_pwm_values_.empty() && pwm_us <= thrust_pwm_values_.back()) {
      RCLCPP_WARN(
          this->get_logger(),
          "Skipping non-increasing PWM row at line %zu (%.3f <= %.3f)",
          line_number,
          pwm_us,
          thrust_pwm_values_.back());
      continue;
    }

    thrust_pwm_values_.push_back(pwm_us);
    thrust_table_newtons_.push_back({force_14v_n, force_16v_n, force_18v_n});
  }

  if (thrust_pwm_values_.size() < 2 || thrust_table_newtons_.size() != thrust_pwm_values_.size()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Invalid thrust table in '%s': need at least 2 valid rows, got %zu",
        csv_path.c_str(),
        thrust_pwm_values_.size());
    return false;
  }

  thrust_table_loaded_ = true;
  RCLCPP_INFO(
      this->get_logger(),
      "Loaded thrust table (%zu rows) from %s, PWM range [%.1f, %.1f]",
      thrust_pwm_values_.size(),
      csv_path.c_str(),
      thrust_pwm_values_.front(),
      thrust_pwm_values_.back());
  return true;
}

double BlueROVBridge::getThrust(double pwm, double voltage) const {
  if (!thrust_table_loaded_ || thrust_pwm_values_.empty()) {
    return 0.0;
  }

  const double clamped_pwm = std::clamp(pwm, thrust_pwm_values_.front(), thrust_pwm_values_.back());
  const double clamped_voltage =
      std::clamp(voltage, thrust_voltage_bins_v_.front(), thrust_voltage_bins_v_.back());

  auto pwm_upper_it = std::lower_bound(thrust_pwm_values_.begin(), thrust_pwm_values_.end(), clamped_pwm);
  size_t pwm_upper_idx = static_cast<size_t>(std::distance(thrust_pwm_values_.begin(), pwm_upper_it));
  if (pwm_upper_idx >= thrust_pwm_values_.size()) {
    pwm_upper_idx = thrust_pwm_values_.size() - 1;
  }
  const size_t pwm_lower_idx = (pwm_upper_idx == 0) ? 0 : pwm_upper_idx - 1;

  const double pwm_low = thrust_pwm_values_[pwm_lower_idx];
  const double pwm_high = thrust_pwm_values_[pwm_upper_idx];
  const double pwm_alpha =
      (pwm_upper_idx == pwm_lower_idx || std::abs(pwm_high - pwm_low) < std::numeric_limits<double>::epsilon())
          ? 0.0
          : (clamped_pwm - pwm_low) / (pwm_high - pwm_low);

  auto voltage_upper_it =
      std::lower_bound(thrust_voltage_bins_v_.begin(), thrust_voltage_bins_v_.end(), clamped_voltage);
  size_t voltage_upper_idx =
      static_cast<size_t>(std::distance(thrust_voltage_bins_v_.begin(), voltage_upper_it));
  if (voltage_upper_idx >= thrust_voltage_bins_v_.size()) {
    voltage_upper_idx = thrust_voltage_bins_v_.size() - 1;
  }
  const size_t voltage_lower_idx = (voltage_upper_idx == 0) ? 0 : voltage_upper_idx - 1;

  const double voltage_low = thrust_voltage_bins_v_[voltage_lower_idx];
  const double voltage_high = thrust_voltage_bins_v_[voltage_upper_idx];
  const double voltage_alpha =
      (voltage_upper_idx == voltage_lower_idx ||
       std::abs(voltage_high - voltage_low) < std::numeric_limits<double>::epsilon())
          ? 0.0
          : (clamped_voltage - voltage_low) / (voltage_high - voltage_low);

  const auto& low_pwm_row = thrust_table_newtons_[pwm_lower_idx];
  const auto& high_pwm_row = thrust_table_newtons_[pwm_upper_idx];
  if (low_pwm_row.size() < kVoltageBinCount || high_pwm_row.size() < kVoltageBinCount) {
    return 0.0;
  }

  const double f00 = low_pwm_row[voltage_lower_idx];
  const double f01 = low_pwm_row[voltage_upper_idx];
  const double f10 = high_pwm_row[voltage_lower_idx];
  const double f11 = high_pwm_row[voltage_upper_idx];

  const double thrust_low_pwm = f00 + voltage_alpha * (f01 - f00);
  const double thrust_high_pwm = f10 + voltage_alpha * (f11 - f10);
  return thrust_low_pwm + pwm_alpha * (thrust_high_pwm - thrust_low_pwm);
}

void BlueROVBridge::handleServoOutputRaw(const mavlink_message_t& msg) {
  if (!thrust_table_loaded_) {
    return;
  }

  mavlink_servo_output_raw_t servo_output_raw;
  mavlink_msg_servo_output_raw_decode(&msg, &servo_output_raw);

  const std::array<uint16_t, kThrusterCount> thruster_pwm = {{
      servo_output_raw.servo1_raw,
      servo_output_raw.servo2_raw,
      servo_output_raw.servo3_raw,
      servo_output_raw.servo4_raw,
      servo_output_raw.servo5_raw,
      servo_output_raw.servo6_raw,
      servo_output_raw.servo7_raw,
      servo_output_raw.servo8_raw}};

  double interpolation_voltage_v = latest_battery_voltage_v_;
  if (!std::isfinite(interpolation_voltage_v)) {
    interpolation_voltage_v = kNominalFallbackVoltageV;
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Battery voltage unavailable. Using " << kNominalFallbackVoltageV << " V for thrust interpolation.");
  }

  forces_actual_msg_.header.stamp = this->now();
  forces_actual_msg_.header.frame_id = "base_link";
  for (size_t i = 0; i < kThrusterCount; ++i) {
    forces_actual_msg_.effort[i] =
        getThrust(static_cast<double>(thruster_pwm[i]), interpolation_voltage_v);
  }
  forcesActualPublisher_->publish(forces_actual_msg_);
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

void BlueROVBridge::handleRawImu(const mavlink_message_t& msg){
  mavlink_raw_imu_t raw_imu;
  mavlink_msg_raw_imu_decode(&msg, &raw_imu);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = imu_frame_id_;

  // Keep existing behavior: use epoch-like time_usec when available, otherwise ROS now().
  if (raw_imu.time_usec > kEpochUsecThreshold) {
    imu_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(raw_imu.time_usec) * 1000LL, RCL_SYSTEM_TIME);
  } else {
    imu_msg.header.stamp = this->now();
  }

    // ArduSub RAW_IMU is body-frame FRD (x-forward, y-right, z-down).
    // ROS IMU is body-frame FLU (x-forward, y-left, z-up).
    // Apply a fixed FRD -> FLU frame rotation using matrix form.
    const Eigen::Matrix3d R = (Eigen::Matrix3d() <<
      -1.0,  0.0,  0.0,
      0.0, 1.0,  0.0,
      0.0,  0.0, -1.0).finished();

    const Eigen::Vector3d acc_frd(
      static_cast<double>(raw_imu.xacc) * kMilligToMps2,
      static_cast<double>(raw_imu.yacc) * kMilligToMps2,
      static_cast<double>(raw_imu.zacc) * kMilligToMps2);
    const Eigen::Vector3d acc_flu = R * acc_frd;
    imu_msg.linear_acceleration.x = acc_flu.x();
    imu_msg.linear_acceleration.y = acc_flu.y();
    imu_msg.linear_acceleration.z = acc_flu.z();

    const Eigen::Vector3d gyro_frd(
      static_cast<double>(raw_imu.xgyro) * kMradpsToRadps,
      static_cast<double>(raw_imu.ygyro) * kMradpsToRadps,
      static_cast<double>(raw_imu.zgyro) * kMradpsToRadps);
    const Eigen::Vector3d gyro_flu =   R * gyro_frd;
    imu_msg.angular_velocity.x = gyro_flu.x();
    imu_msg.angular_velocity.y = gyro_flu.y();
    imu_msg.angular_velocity.z = gyro_flu.z();

  // Orientation is unknown for RAW_IMU: publish a valid identity quaternion and mark unknown.
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = -1.0;

  // Unknown covariance; do not use these covariance fields yet.
  imu_msg.angular_velocity_covariance.fill(-1.0);
  imu_msg.linear_acceleration_covariance.fill(-1.0);

  imuPublisher_->publish(imu_msg);
}

void BlueROVBridge::handleScaledPressure2(const mavlink_message_t& msg){
  mavlink_scaled_pressure2_t pressure2;
  mavlink_msg_scaled_pressure2_decode(&msg, &pressure2);

  sensor_msgs::msg::FluidPressure pressure_msg;
  pressure_msg.header.stamp = this->now();
  pressure_msg.header.frame_id = pressure_frame_id_;
  pressure_msg.fluid_pressure = static_cast<double>(pressure2.press_abs) * 100.0;  // hPa -> Pa
  pressure_msg.variance = 0.0;

  pressureScaled2Publisher_->publish(pressure_msg);
}

void BlueROVBridge::handleGimbalStatus(const mavlink_message_t& msg){
  mavlink_gimbal_device_attitude_status_t gimbal_status;
  mavlink_msg_gimbal_device_attitude_status_decode(&msg, &gimbal_status);

  auto gimbal_status_msg = std::make_unique<auv_core_helper::msg::GimbalStatus>();
  gimbal_status_msg->header.stamp = now();
  gimbal_status_msg->flags = gimbal_status.flags;

  tf2::Quaternion q(gimbal_status.q[1], gimbal_status.q[2], gimbal_status.q[3], gimbal_status.q[0]);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  gimbal_status_msg->roll = static_cast<float>(roll * 180.0 / M_PI);
  gimbal_status_msg->pitch = static_cast<float>(pitch * 180.0 / M_PI);
  gimbal_status_msg->yaw = static_cast<float>(yaw * 180.0 / M_PI);

  gimbalStatusPublisher_->publish(*gimbal_status_msg);
}

void BlueROVBridge::safetySwitchCallback(const std_msgs::msg::Bool::SharedPtr msg){
  failsafe_active_ = msg->data;

  if (failsafe_active_){
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Failsafe active! Putting vehicle to POSHOLD mode, disarming vehicle and rejecting control commands.");
    setFlightMode("POSHOLD");
    setArmState(false);
    cycleServo(14,1900,1000,3);

  } else if (!failsafe_active_){
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Failsafe inactive. Vehicle control commands are now accepted.");
  }
}

void BlueROVBridge::armingServiceCallback(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<SetBoolSrv::Request> request)
{
  auto resp = std::make_shared<SetBoolSrv::Response>();

   if (failsafe_active_) {
    resp->success = false;
    resp->message = "Failsafe active — arming rejected!";
    armingService_->send_response(*header, *resp);
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Arming command rejected due to failsafe.");
    return;
  }

  setArmState(request->data);

  resp->success = false;
  resp->message = "Timed out.";

  pending_arm_ = PendingArm{header, resp, request->data, this->now() + kSrvTimeout};
}

void BlueROVBridge::setArmState(bool arm_vehicle)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN(this->get_logger(), 
        "Cannot %s yet; no autopilot heartbeat discovered!", arm_vehicle ? "arm" : "disarm");
    return;
  }

  RCLCPP_INFO(this->get_logger(),"%s vehicle (sys=%d, comp=%d)...",arm_vehicle ? "Arming" : "Disarming",target_system_, target_component_);

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

void BlueROVBridge::flightModeServiceCallback(const std::shared_ptr<rmw_request_id_t> header,
                                              const std::shared_ptr<SetModeSrv::Request> request)
{
  auto resp = std::make_shared<SetModeSrv::Response>();

  if (failsafe_active_) {
    resp->success = false;
    resp->message = "Failsafe active — flight mode change rejected!";
    flightModeService_->send_response(*header, *resp);
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Flight mode change rejected due to failsafe.");
    return;
  }

  int32_t custom = mapModeStringToNumber(request->mode);
  setFlightMode(request->mode);

  resp->success = false;
  resp->message = "Timed out.";

  pending_mode_ = PendingMode{header, resp, custom,this->now() + kSrvTimeout};
}

int32_t BlueROVBridge::mapModeStringToNumber(const std::string & mode) const
{
  if (mode ==  auv_core_helper::FlightMode::STABILIZE) return 0;
  if (mode ==  auv_core_helper::FlightMode::ALT_HOLD)  return 2;
  if (mode ==  auv_core_helper::FlightMode::GUIDED)    return 4;
  if (mode ==  auv_core_helper::FlightMode::SURFACE)   return 9;
  if (mode ==  auv_core_helper::FlightMode::POSHOLD)   return 16;
  if (mode ==  auv_core_helper::FlightMode::SURFTRAK)  return 21;
  /* default → MANUAL */
  return 19;
}

void BlueROVBridge::setFlightMode(const std::string& mode)
{
  // Map mode string to ArduSub custom mode number
  int32_t custom_mode = mapModeStringToNumber(mode);

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
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Setting flight-mode to "<< mode.c_str());
}

void BlueROVBridge::rcChannelsOverride(uint16_t rc[]){

  mavlink_message_t msg;
  mavlink_msg_rc_channels_override_pack(
    system_id_,
    component_id_,
    &msg,
    target_system_,
    target_component_,
    rc[0],
    rc[1],
    rc[2],
    rc[3],
    rc[4],
    rc[5],
    rc[6],
    rc[7],
    rc[8],
    rc[9],
    rc[10],
    rc[11],
    rc[12],
    rc[13],
    rc[14],
    rc[15],
    rc[16],
    rc[17]
  );
  sendMavlinkMessage(msg);
}

void BlueROVBridge::setServo(uint8_t servoID,uint16_t pwm){

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    system_id_,
    component_id_,
    &msg,
    target_system_,
    target_component_,
    MAV_CMD_DO_SET_SERVO,
    0,
    servoID,
    pwm,
    0,0,0,0,0  //param3-7 unused
  );
  sendMavlinkMessage(msg);
  // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Setting servo "<< static_cast<int>(servoID) <<" to "<< pwm);
}

void BlueROVBridge::cycleServo(uint8_t servoID,uint16_t pwm,uint16_t cycleCount,uint16_t cycleTime){

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    system_id_,
    component_id_,
    &msg,
    target_system_,
    target_component_,
    MAV_CMD_DO_REPEAT_SERVO,
    0,
    servoID,
    pwm,
    cycleCount,
    cycleTime,
    0,0,0  //param5-7 unused
  );
  sendMavlinkMessage(msg);
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Cycling servo "<< static_cast<int>(servoID) <<" to "<< pwm);
}

void BlueROVBridge::setGlobalOriginServiceCallback(const std::shared_ptr<auv_core_helper::srv::SetGlobalOrigin::Request> request,
                                                   std::shared_ptr<auv_core_helper::srv::SetGlobalOrigin::Response> response){
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Setting global origin service called: " << request->latitude << ", " << request->longitude << ", " << request->altitude);
  mavlink_set_gps_global_origin_t set_gps_global_origin;
  set_gps_global_origin.latitude = static_cast<int32_t>(request->latitude * 1e7);
  set_gps_global_origin.longitude = static_cast<int32_t>(request->longitude * 1e7);
  set_gps_global_origin.altitude = static_cast<int32_t>(request->altitude * 1000.0);
  setGlobalOrigin(set_gps_global_origin);

  if(gps_global_origin.latitude == set_gps_global_origin.latitude && gps_global_origin.longitude == set_gps_global_origin.longitude && gps_global_origin.altitude == set_gps_global_origin.altitude) {
      response->success = true;
      response->message = "Global origin set.";
    } else {
      response->success = false;
      response->message = "Global origin set failed.";
    }
}

void BlueROVBridge::setGlobalOrigin(mavlink_set_gps_global_origin_t& set_gps_global_origin){
  mavlink_message_t msg;
  mavlink_msg_set_gps_global_origin_pack(
      system_id_,
      component_id_,
      &msg,
      target_system_,
      set_gps_global_origin.latitude,
      set_gps_global_origin.longitude,
      set_gps_global_origin.altitude,
      this->now().nanoseconds()
    );
  sendMavlinkMessage(msg);
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Global origin set message sent to"
  << target_system_<<","<<(double)set_gps_global_origin.latitude/1e7 <<","<<(double)set_gps_global_origin.longitude/1e7 <<","<<(double)set_gps_global_origin.altitude/1000.0);
}


void BlueROVBridge::gimbalServiceCallback(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<auv_core_helper::srv::SetGimbalAttitude::Request> request){
  
  float pitch = request->pitch * 180.0 / M_PI;
  float yaw = request->yaw * 180.0 / M_PI;  
  
  setGimbalAttitude(pitch, yaw);        // The mavlink command expect the pitch and yaw in degrees

  auto response = std::make_shared<auv_core_helper::srv::SetGimbalAttitude::Response>();
  response->success = false;
  response->message = "Timed out.";

  pending_gimbal_attitude_ = PendingGimbalAttitude{header, response, request, this->now() + kSrvTimeout};
}

void BlueROVBridge::setGimbalAttitude(float gimbal_pitch, float gimbal_yaw){
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    system_id_, 
    component_id_,
    &msg,
    target_system_,
    target_component_,
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    0,
    gimbal_pitch,
    gimbal_yaw, 
    std::numeric_limits<float>::quiet_NaN(),  //Pitch rate in deg/s (positive is up) or NaN if unused
    std::numeric_limits<float>::quiet_NaN(),  //Yaw rate in deg/s (positive is clockwise) or NaN if unused
    0,                                        //Flags (0=Yaw is body-frame/follow, 16=Yaw is earth-frame/lock)
    0,                                        //not used param
    0                                         //Gimbal device ID (0 is primary gimbal, 1 is 1st gimbal, 2 is 2nd gimbal)
  );
  sendMavlinkMessage(msg);
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Gimbal attitude set to pitch: " << gimbal_pitch << ", yaw: " << gimbal_yaw);
}

void BlueROVBridge::desiredCtrlModeCallback(const std_msgs::msg::String::SharedPtr msg){
  ctrlMode =  msg->data;
}

void BlueROVBridge::globalPoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg){
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
  velocityGoalGlobal << msg->linear.x,
                           msg->linear.y,
                           msg->linear.z, 
                           msg->angular.x,
                           msg->angular.y,
                           msg->angular.z;

   auto almost_equal = [this](double a, double b, double eps){
      return std::fabs(a - b) < eps;
  };

  velGoalGlobalChanged =
        !almost_equal(velocityGoalGlobal(0), velGoalGlobalLast(0), VELX_EPS) ||
        !almost_equal(velocityGoalGlobal(1), velGoalGlobalLast(1), VELY_EPS) ||
        !almost_equal(velocityGoalGlobal(2), velGoalGlobalLast(2), VELZ_EPS) ||
        !almost_equal(velocityGoalGlobal(3), velGoalGlobalLast(3), ANGX_EPS) ||
        !almost_equal(velocityGoalGlobal(4), velGoalGlobalLast(4), ANGY_EPS) ||
        !almost_equal(velocityGoalGlobal(5), velGoalGlobalLast(5), ANGZ_EPS);
}

void BlueROVBridge::Execute(){

  /* TIMEOUT ARM */
  if (pending_arm_ && this->now() > pending_arm_->deadline) {
          armingService_->send_response(*pending_arm_->header, *pending_arm_->resp);
          RCLCPP_WARN(get_logger(), "Arming/disarming request timed out.");
          pending_arm_.reset();
  }

  /* TIMEOUT MODE */
  if (pending_mode_ && this->now() > pending_mode_->deadline) {
          flightModeService_->send_response(*pending_mode_->header, *pending_mode_->resp);
          RCLCPP_WARN(get_logger(), "Flight-mode change timed out.");
          pending_mode_.reset();
  } 

  /* TIMEOUT GIMBAL */
  if (pending_gimbal_attitude_ && this->now() > pending_gimbal_attitude_->deadline) {
          gimbalService_->send_response(*pending_gimbal_attitude_->header, *pending_gimbal_attitude_->resp);
          RCLCPP_WARN(get_logger(), "Gimbal attitude change timed out.");
          pending_gimbal_attitude_.reset();
  } 

  if (global_pose_msg && global_velocity_msg && got_heartbeat_) {
      globalPoseActualPublisher_->publish(*global_pose_msg);
      globalVelocityActualPublisher_->publish(*global_velocity_msg);

      if (poseGoalGlobalChanged || velGoalGlobalChanged){        // If the pose or velocity goal has changed, send the new goal to the autopilot

        if (failsafe_active_){
          RCLCPP_WARN_STREAM_THROTTLE(get_logger(),*get_clock(),1000, "Failsafe active, not sending goals.");
          return; 
         }

        if(ctrlMode == auv_core_helper::BrigdeMode::PoseCtrl){
          position_target_global_.type_mask =
                POSITION_TARGET_TYPEMASK_VX_IGNORE  |
                POSITION_TARGET_TYPEMASK_VY_IGNORE  |
                POSITION_TARGET_TYPEMASK_VZ_IGNORE  |
                POSITION_TARGET_TYPEMASK_AX_IGNORE  |
                POSITION_TARGET_TYPEMASK_AY_IGNORE  |
                POSITION_TARGET_TYPEMASK_AZ_IGNORE  |
                POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
        }
        else if (ctrlMode == auv_core_helper::BrigdeMode::VelCtrl){
          position_target_global_.type_mask =
                POSITION_TARGET_TYPEMASK_X_IGNORE  |
                POSITION_TARGET_TYPEMASK_Y_IGNORE  |
                POSITION_TARGET_TYPEMASK_Z_IGNORE  |
                POSITION_TARGET_TYPEMASK_AX_IGNORE |
                POSITION_TARGET_TYPEMASK_AY_IGNORE |
                POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
        }
        else {
          position_target_global_.type_mask =
                POSITION_TARGET_TYPEMASK_X_IGNORE  |
                POSITION_TARGET_TYPEMASK_Y_IGNORE  |
                POSITION_TARGET_TYPEMASK_Z_IGNORE  |
                POSITION_TARGET_TYPEMASK_VX_IGNORE |
                POSITION_TARGET_TYPEMASK_VY_IGNORE |
                POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                POSITION_TARGET_TYPEMASK_AX_IGNORE |
                POSITION_TARGET_TYPEMASK_AY_IGNORE |
                POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                POSITION_TARGET_TYPEMASK_YAW_IGNORE|
                POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
        }
        
        position_target_global_.time_boot_ms     = static_cast<uint32_t>(this->now().nanoseconds() / 1e6);
        position_target_global_.target_system    = target_system_;
        position_target_global_.target_component = target_component_;
        position_target_global_.coordinate_frame = MAV_FRAME_GLOBAL_INT;            
        position_target_global_.lat_int = static_cast<int32_t>(poseGoalGlobal(0) * 1e7);       // deg → 1e-7°
        position_target_global_.lon_int = static_cast<int32_t>(poseGoalGlobal(1) * 1e7);
        position_target_global_.alt     = poseGoalGlobal(2);
        position_target_global_.yaw      = static_cast<float>(poseGoalGlobal(5));
        position_target_global_.yaw_rate = 0.0f;
        position_target_global_.vx = velocityGoalGlobal(0);
        position_target_global_.vy = velocityGoalGlobal(1);
        position_target_global_.vz = velocityGoalGlobal(2);

        double yaw_deg = poseGoalGlobal(5) * 180.0 / M_PI;
        if (yaw_deg < 0.0)
          yaw_deg += 360.0; // Ensure yaw is in [0, 360) range
        condition_yaw_.param1 = yaw_deg; // Set the yaw angle in degrees

        // Send the condition yaw and the position target to the autopilot
        sendConditionYaw(condition_yaw_);
        SetPositionTargetGlobalInt(position_target_global_);
        
        poseGoalGlobalLast = poseGoalGlobal;
        poseGoalGlobalChanged = false; // Reset the flag
        velGoalGlobalLast = velocityGoalGlobal;
        velGoalGlobalChanged = false; // Reset the flag
      }
    }
}

void BlueROVBridge::SetPositionTargetGlobalInt(const mavlink_set_position_target_global_int_t& position_target_global_)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
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

void BlueROVBridge::sendConditionYaw(const mavlink_command_long_t& condition_yaw_)
{
  if (!got_heartbeat_) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
     "Cannot send CONDITION_YAW command; no autopilot heartbeat discovered!");
    return;
  }
  
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Setting vehicle heading: " << condition_yaw_.param1 << " degrees");
    
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
      1.0f,                       // param3: direction: -1=CCW, 1=CW, 0=shortest
      0.0f,                       // param4: 0=absolute, 1=relative
      0.0f, 0.0f, 0.0f);         // param5-7: unused
  
  // Send the message
  sendMavlinkMessage(msg);
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,"CONDITION_YAW command sent");
}
