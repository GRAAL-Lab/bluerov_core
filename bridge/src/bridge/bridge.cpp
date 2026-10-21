#include "bluerov-bridge/bluerov_bridge.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#ifdef BRIDGE_ENABLE_CAMERA
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <sensor_msgs/image_encodings.hpp>
#endif

// C / C++ Includes
#include <chrono>
#include <cctype>
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
constexpr double kMilligaussToTesla = 1.0e-7;
constexpr double kNominalFallbackVoltageV = 16.0;
// MAVLink/ArduPilot aircraft body axes are forward-right-down (FRD), while
// ROS REP-103 base_link axes are forward-left-up (FLU). This is a pi rotation
// about the shared forward X axis: v_flu = R_flu_frd * v_frd.
const Eigen::Matrix3d kFrdToFlu = (Eigen::Matrix3d() <<
     1.0,  0.0,  0.0,
     0.0, -1.0,  0.0,
     0.0,  0.0, -1.0).finished();
#ifdef BRIDGE_ENABLE_CAMERA
constexpr char kCameraAppsinkName[] = "bridge_camera_sink";
const auto kCameraSampleTimeout = std::chrono::milliseconds(100);
const auto kCameraRestartDelay = std::chrono::seconds(2);
std::once_flag kGStreamerInitOnce;

void ensureGStreamerInitialized() {
  std::call_once(kGStreamerInitOnce, []() {
    gst_init(nullptr, nullptr);
  });
}

std::string escapeGStreamerString(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

const char* toGstBoolean(const bool value) {
  return value ? "true" : "false";
}
#endif

#ifdef BRIDGE_ENABLE_CAMERA
std::string toLowerCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}
#endif

std::string trim(const std::string& input) {
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

#ifdef BRIDGE_ENABLE_CAMERA
std::string normalizeCameraOutputEncoding(const std::string& encoding) {
  const std::string normalized = toLowerCopy(trim(encoding));
  if (normalized == sensor_msgs::image_encodings::RGB8 ||
      normalized == sensor_msgs::image_encodings::BGR8 ||
      normalized == sensor_msgs::image_encodings::RGBA8) {
    return normalized;
  }
  return sensor_msgs::image_encodings::RGBA8;
}

bool isRtspUri(const std::string& uri) {
  const std::string normalized = toLowerCopy(trim(uri));
  return normalized.rfind("rtsp://", 0) == 0;
}
#endif

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

const rclcpp::Duration BlueROVBridge::HeartbeatTimeout = rclcpp::Duration::from_seconds(4.0);

/**
 * @file bluerov_bridge.cpp
 * @brief Implementation of the BlueROVBridge class that connects ROS2 to ArduSub via MAVLink
 * Communication is via MAVLink over UDP, using the standard ArduSub protocol.
 */
BlueROVBridge::BlueROVBridge(const rclcpp::NodeOptions& options): Node("mavlink_bridge", options){
  int configured_system_id = static_cast<int>(system_id_);
  int configured_component_id = static_cast<int>(component_id_);

  this->declare_parameter<bool>("simulation_mode", simulation_mode_);
  this->declare_parameter<std::string>("remote_address", remote_addr_str_);
  this->declare_parameter<int>("system_id", configured_system_id);
  this->declare_parameter<int>("component_id", configured_component_id);
  this->declare_parameter<int>("port", port_);
  this->declare_parameter<std::string>("velocity_actual_topic", velocity_actual_topic_);
  this->declare_parameter<std::string>("dvl_distance_topic", dvl_distance_topic_);
  system_id_ = static_cast<uint8_t>(configured_system_id);
  component_id_ = static_cast<uint8_t>(configured_component_id);

  this->declare_parameter<std::string>("imu_topic", imu_topic_);
  this->declare_parameter<std::string>("magnetic_field_topic", magnetic_field_topic_);
  this->declare_parameter<std::string>("imu_frame_id", imu_frame_id_);
  this->declare_parameter<double>("imu_rate_hz", static_cast<double>(imu_rate_hz_));
  this->declare_parameter<double>("pressure_rate_hz", static_cast<double>(pressure_rate_hz_));
  this->declare_parameter<double>("servo_output_rate_hz", static_cast<double>(servo_output_rate_hz_));
  this->declare_parameter<std::string>("forces_actual_topic", forces_actual_topic_);
  this->declare_parameter<std::string>("pressure_topic", pressure_topic_);
  this->declare_parameter<std::string>("pressure_frame_id", pressure_frame_id_);
  this->declare_parameter<std::string>("thrust_table_csv_path", getDefaultThrustCsvPath());
  this->declare_parameter<bool>("camera_enabled", camera_config_.enabled);
  this->declare_parameter<std::string>("camera_source_uri", camera_config_.source_uri);
  this->declare_parameter<std::string>("camera_rtp_caps", camera_config_.rtp_caps);
  this->declare_parameter<std::string>("camera_topic", camera_config_.topic);
  this->declare_parameter<std::string>("camera_info_topic", camera_config_.info_topic);
  this->declare_parameter<std::string>("camera_frame_id", camera_config_.frame_id);
  this->declare_parameter<bool>("camera_use_hw_decoder", camera_config_.use_hw_decoder);
  this->declare_parameter<bool>("camera_qos_reliable", camera_config_.qos_reliable);
  this->declare_parameter<bool>("camera_enable_max_performance", camera_config_.enable_max_performance);
  this->declare_parameter<int>("camera_preview_width", camera_config_.preview_width);
  this->declare_parameter<int>("camera_preview_height", camera_config_.preview_height);
  this->declare_parameter<double>("camera_preview_max_fps", camera_config_.preview_max_fps);
  this->declare_parameter<std::string>("camera_output_encoding", camera_config_.output_encoding);
  this->declare_parameter<double>("camera_info_publish_rate_hz", camera_config_.camera_info_publish_rate_hz);
  this->declare_parameter<int>("camera_rtp_latency_ms", camera_config_.rtp_latency_ms);
  this->declare_parameter<int>("camera_udp_buffer_size_bytes", camera_config_.udp_buffer_size_bytes);
  this->declare_parameter<int>("camera_appsink_max_buffers", camera_config_.appsink_max_buffers);
  this->declare_parameter<bool>("camera_appsink_drop", camera_config_.appsink_drop);
  this->declare_parameter<std::string>("camera_pipeline_override", camera_config_.pipeline_override);
  this->get_parameter("simulation_mode", simulation_mode_);
  this->get_parameter("remote_address", remote_addr_str_);
  this->get_parameter("system_id", configured_system_id);
  this->get_parameter("component_id", configured_component_id);
  this->get_parameter("port", port_);
  this->get_parameter("velocity_actual_topic", velocity_actual_topic_);
  this->get_parameter("dvl_distance_topic", dvl_distance_topic_);
  system_id_ = static_cast<uint8_t>(configured_system_id);
  component_id_ = static_cast<uint8_t>(configured_component_id);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("magnetic_field_topic", magnetic_field_topic_);
  this->get_parameter("imu_frame_id", imu_frame_id_);
  imu_rate_hz_ = static_cast<float>(this->get_parameter("imu_rate_hz").as_double());
  pressure_rate_hz_ = static_cast<float>(this->get_parameter("pressure_rate_hz").as_double());
  servo_output_rate_hz_ = static_cast<float>(this->get_parameter("servo_output_rate_hz").as_double());
  this->get_parameter("forces_actual_topic", forces_actual_topic_);
  this->get_parameter("pressure_topic", pressure_topic_);
  this->get_parameter("pressure_frame_id", pressure_frame_id_);
  this->get_parameter("thrust_table_csv_path", thrust_table_csv_path_);
  this->get_parameter("camera_enabled", camera_config_.enabled);
  this->get_parameter("camera_source_uri", camera_config_.source_uri);
  this->get_parameter("camera_rtp_caps", camera_config_.rtp_caps);
  this->get_parameter("camera_topic", camera_config_.topic);
  this->get_parameter("camera_info_topic", camera_config_.info_topic);
  this->get_parameter("camera_frame_id", camera_config_.frame_id);
  this->get_parameter("camera_use_hw_decoder", camera_config_.use_hw_decoder);
  this->get_parameter("camera_qos_reliable", camera_config_.qos_reliable);
  this->get_parameter("camera_enable_max_performance", camera_config_.enable_max_performance);
  this->get_parameter("camera_preview_width", camera_config_.preview_width);
  this->get_parameter("camera_preview_height", camera_config_.preview_height);
  this->get_parameter("camera_preview_max_fps", camera_config_.preview_max_fps);
  this->get_parameter("camera_output_encoding", camera_config_.output_encoding);
  this->get_parameter("camera_info_publish_rate_hz", camera_config_.camera_info_publish_rate_hz);
  this->get_parameter("camera_rtp_latency_ms", camera_config_.rtp_latency_ms);
  this->get_parameter("camera_udp_buffer_size_bytes", camera_config_.udp_buffer_size_bytes);
  this->get_parameter("camera_appsink_max_buffers", camera_config_.appsink_max_buffers);
  this->get_parameter("camera_appsink_drop", camera_config_.appsink_drop);
  this->get_parameter("camera_pipeline_override", camera_config_.pipeline_override);
  camera_config_.preview_width = std::max(camera_config_.preview_width, 0);
  camera_config_.preview_height = std::max(camera_config_.preview_height, 0);
  camera_config_.preview_max_fps = std::max(camera_config_.preview_max_fps, 0.0);
#ifdef BRIDGE_ENABLE_CAMERA
  camera_config_.output_encoding = normalizeCameraOutputEncoding(camera_config_.output_encoding);
#endif
  camera_config_.camera_info_publish_rate_hz = std::max(camera_config_.camera_info_publish_rate_hz, 0.0);
  camera_config_.rtp_latency_ms = std::max(camera_config_.rtp_latency_ms, 0);
  camera_config_.udp_buffer_size_bytes = std::max(camera_config_.udp_buffer_size_bytes, 0);
  camera_config_.appsink_max_buffers = std::max(camera_config_.appsink_max_buffers, 1);

  RCLCPP_INFO(this->get_logger(), "Starting BlueROVBridge node (UDP port %d, remote_addr: %s, sysid: %d, compid: %d)",
              port_, remote_addr_str_.c_str(), system_id_, component_id_);

  if (!loadThrustTableFromCsv(thrust_table_csv_path_)) {
    throw std::runtime_error("Failed loading thrust table CSV from: " + thrust_table_csv_path_);
  }

  // Initialize the MAVLink UDP connection on local port
  initMavlinkConnection();

  // Setup ROS pubs/subs
  globalVelocityActualPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(velocity_actual_topic_,1);
  dvlDistancePublisher_ = this->create_publisher<std_msgs::msg::Float64>(dvl_distance_topic_,1);
  imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 20);
  magneticFieldPublisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(magnetic_field_topic_, 20);
  forcesActualPublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(forces_actual_topic_, 10);
  pressureScaled2Publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(pressure_topic_, 20);
#ifdef BRIDGE_ENABLE_CAMERA
  if (camera_config_.enabled) {
    auto camera_qos = rclcpp::SensorDataQoS().keep_last(1);
    camera_qos = camera_config_.qos_reliable ? camera_qos.reliable() : camera_qos.best_effort();
    cameraImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>(camera_config_.topic, camera_qos);
    cameraInfoPublisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_config_.info_topic,
        camera_config_.qos_reliable ? rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
                                    : rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  }
#endif

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

  // Timers
  mavlink_heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BlueROVBridge::sendMavlinkHeartbeat, this));
  ardusub_heartbeat_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BlueROVBridge::ardusubHeartbeatWatchdog, this)); // ~1Hz
  // Poll faster than the highest configured sensor stream. The callback drains
  // the non-blocking UDP socket, so this keeps 200 Hz IMU samples from being
  // forwarded to ROS in 10 ms batches.
  data_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&BlueROVBridge::receiveData, this));

  last_heartbeat_time_ = this->now();

#ifdef BRIDGE_ENABLE_CAMERA
  if (camera_config_.enabled) {
    if (camera_config_.topic.empty() || camera_config_.info_topic.empty()) {
      RCLCPP_WARN(this->get_logger(), "Camera stream requested but a camera topic is empty. Disabling camera publisher.");
      camera_config_.enabled = false;
      cameraImagePublisher_.reset();
      cameraInfoPublisher_.reset();
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Camera stream enabled (%s decoder, %s QoS, %s output) %s -> %s",
          camera_config_.use_hw_decoder ? "Jetson HW" : "CPU",
          camera_config_.qos_reliable ? "reliable" : "best-effort",
          camera_config_.output_encoding.c_str(),
          camera_config_.source_uri.c_str(),
          camera_config_.topic.c_str());
      RCLCPP_INFO(
          this->get_logger(),
          "Camera preview configured at %dx%d <= %.1f fps, camera info %.1f Hz, RTP latency %d ms",
          camera_config_.preview_width,
          camera_config_.preview_height,
          camera_config_.preview_max_fps,
          camera_config_.camera_info_publish_rate_hz,
          camera_config_.rtp_latency_ms);
      startCameraStream();
    }
  }
#endif
}

BlueROVBridge::~BlueROVBridge(){
#ifdef BRIDGE_ENABLE_CAMERA
  stopCameraStream();
#endif
  if (sock_fd_ != -1) {
    close(sock_fd_);
  }
  RCLCPP_INFO(this->get_logger(), "BlueROVBridge node shutting down.");
}

#ifdef BRIDGE_ENABLE_CAMERA
void BlueROVBridge::startCameraStream() {
  if (!camera_config_.enabled || camera_thread_.joinable()) {
    return;
  }

  ensureGStreamerInitialized();
  camera_stop_requested_.store(false);
  camera_thread_ = std::thread(&BlueROVBridge::cameraStreamLoop, this);
}

void BlueROVBridge::stopCameraStream() {
  camera_stop_requested_.store(true);
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }
  destroyCameraPipeline();
}

bool BlueROVBridge::createCameraPipeline(const bool use_hw_decoder) {
  const std::string pipeline_description = buildCameraPipelineDescription(use_hw_decoder);
  GError* error = nullptr;
  GstElement* pipeline = gst_parse_launch(pipeline_description.c_str(), &error);
  if (pipeline == nullptr) {
    const std::string error_message = error != nullptr ? error->message : "unknown parse error";
    RCLCPP_ERROR(this->get_logger(), "Failed to create camera pipeline: %s", error_message.c_str());
    if (error != nullptr) {
      g_error_free(error);
    }
    return false;
  }

  GstElement* appsink = gst_bin_get_by_name(GST_BIN(pipeline), kCameraAppsinkName);
  if (appsink == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Camera pipeline is missing the appsink named '%s'.", kCameraAppsinkName);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return false;
  }

  gst_app_sink_set_emit_signals(GST_APP_SINK(appsink), FALSE);
  gst_app_sink_set_drop(GST_APP_SINK(appsink), camera_config_.appsink_drop);
  gst_app_sink_set_max_buffers(GST_APP_SINK(appsink), static_cast<guint>(camera_config_.appsink_max_buffers));
  gst_app_sink_set_wait_on_eos(GST_APP_SINK(appsink), FALSE);

  if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(this->get_logger(), "Camera pipeline failed to enter the PLAYING state.");
    gst_object_unref(appsink);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return false;
  }

  std::lock_guard<std::mutex> lock(camera_pipeline_mutex_);
  camera_pipeline_ = pipeline;
  camera_appsink_ = appsink;
  return true;
}

void BlueROVBridge::destroyCameraPipeline() {
  GstElement* pipeline = nullptr;
  GstElement* appsink = nullptr;
  {
    std::lock_guard<std::mutex> lock(camera_pipeline_mutex_);
    pipeline = camera_pipeline_;
    appsink = camera_appsink_;
    camera_pipeline_ = nullptr;
    camera_appsink_ = nullptr;
  }

  if (pipeline != nullptr) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
  }
  if (appsink != nullptr) {
    gst_object_unref(appsink);
  }
  if (pipeline != nullptr) {
    gst_object_unref(pipeline);
  }
}

std::string BlueROVBridge::buildCameraPipelineDescription(const bool use_hw_decoder) const {
  if (!camera_config_.pipeline_override.empty()) {
    return camera_config_.pipeline_override;
  }

  const std::string output_encoding = normalizeCameraOutputEncoding(camera_config_.output_encoding);
  const std::string cpu_output_format =
      output_encoding == sensor_msgs::image_encodings::RGB8
          ? "RGB"
          : (output_encoding == sensor_msgs::image_encodings::BGR8 ? "BGR" : "RGBA");
  const std::string hw_output_format =
      output_encoding == sensor_msgs::image_encodings::BGR8 ? "BGRx" : "RGBA";

  std::ostringstream pipeline;
  if (isRtspUri(camera_config_.source_uri)) {
    pipeline << "rtspsrc location=\"" << escapeGStreamerString(camera_config_.source_uri) << "\""
             << " latency=" << camera_config_.rtp_latency_ms
             << " drop-on-latency=true protocols=tcp ntp-sync=false"
             << " ! rtph264depay"
             << " ! h264parse config-interval=-1";
  } else {
    pipeline << "udpsrc uri=\"" << escapeGStreamerString(camera_config_.source_uri) << "\""
             << " buffer-size=" << camera_config_.udp_buffer_size_bytes
             << " do-timestamp=true"
             << " retrieve-sender-address=false"
             << " caps=\"" << escapeGStreamerString(camera_config_.rtp_caps) << "\""
             << " ! rtpjitterbuffer latency=" << camera_config_.rtp_latency_ms
             << " drop-on-latency=true faststart-min-packets=1"
             << " ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0"
             << " ! rtph264depay"
             << " ! h264parse config-interval=-1";
  }

  if (use_hw_decoder) {
    pipeline << " ! nvv4l2decoder disable-dpb=true enable-max-performance="
             << toGstBoolean(camera_config_.enable_max_performance)
             << " ! nvvidconv";
    if (camera_config_.preview_width > 0 && camera_config_.preview_height > 0) {
      pipeline << " ! video/x-raw,format=" << hw_output_format
               << ",width=" << camera_config_.preview_width
               << ",height=" << camera_config_.preview_height;
    } else {
      pipeline << " ! video/x-raw,format=" << hw_output_format;
    }
  } else {
    pipeline << " ! avdec_h264 max-threads=2"
             << " ! videoconvert"
             << " ! videoscale";
    if (camera_config_.preview_width > 0 && camera_config_.preview_height > 0) {
      pipeline << " ! video/x-raw,format=" << cpu_output_format
               << ",width=" << camera_config_.preview_width
               << ",height=" << camera_config_.preview_height;
    } else {
      pipeline << " ! video/x-raw,format=" << cpu_output_format;
    }
  }

  pipeline << " ! appsink name=" << kCameraAppsinkName
           << " sync=false"
           << " max-buffers=" << camera_config_.appsink_max_buffers
           << " drop=" << toGstBoolean(camera_config_.appsink_drop);
  return pipeline.str();
}

bool BlueROVBridge::publishCameraSample(GstSample* sample) {
  if (sample == nullptr || cameraImagePublisher_ == nullptr) {
    return false;
  }

  const bool has_image_subscribers =
      cameraImagePublisher_->get_subscription_count() > 0 ||
      cameraImagePublisher_->get_intra_process_subscription_count() > 0;
  const bool has_info_subscribers =
      cameraInfoPublisher_ != nullptr &&
      (cameraInfoPublisher_->get_subscription_count() > 0 ||
       cameraInfoPublisher_->get_intra_process_subscription_count() > 0);
  if (!has_image_subscribers && !has_info_subscribers) {
    return false;
  }

  if (camera_config_.preview_max_fps > 0.0) {
    const auto now_steady = std::chrono::steady_clock::now();
    const auto min_period = std::chrono::duration<double>(1.0 / camera_config_.preview_max_fps);
    if (last_camera_publish_time_.time_since_epoch().count() != 0 &&
        now_steady - last_camera_publish_time_ < min_period) {
      return false;
    }
  }

  GstCaps* caps = gst_sample_get_caps(sample);
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  if (caps == nullptr || buffer == nullptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Camera sample arrived without caps or payload.");
    return false;
  }

  GstVideoInfo video_info;
  if (!gst_video_info_from_caps(&video_info, caps)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to read camera caps into a video frame description.");
    return false;
  }

  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &video_info, buffer, GST_MAP_READ)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to map the decoded camera frame.");
    return false;
  }

  size_t input_channel_count = 0;
  size_t output_channel_count = 0;
  std::string encoding;
  bool drop_alpha_channel = false;
  switch (GST_VIDEO_INFO_FORMAT(&video_info)) {
    case GST_VIDEO_FORMAT_RGB:
      input_channel_count = 3;
      output_channel_count = 3;
      encoding = sensor_msgs::image_encodings::RGB8;
      break;
    case GST_VIDEO_FORMAT_BGR:
      input_channel_count = 3;
      output_channel_count = 3;
      encoding = sensor_msgs::image_encodings::BGR8;
      break;
    case GST_VIDEO_FORMAT_RGBA:
      input_channel_count = 4;
      output_channel_count = 4;
      encoding = sensor_msgs::image_encodings::RGBA8;
      break;
    case GST_VIDEO_FORMAT_RGBx:
      input_channel_count = 4;
      output_channel_count = 3;
      drop_alpha_channel = true;
      encoding = sensor_msgs::image_encodings::RGB8;
      break;
    case GST_VIDEO_FORMAT_BGRA:
      input_channel_count = 4;
      output_channel_count = 4;
      encoding = sensor_msgs::image_encodings::BGRA8;
      break;
    case GST_VIDEO_FORMAT_BGRx:
      input_channel_count = 4;
      output_channel_count = 3;
      drop_alpha_channel = true;
      encoding = sensor_msgs::image_encodings::BGR8;
      break;
    default:
      gst_video_frame_unmap(&frame);
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Unsupported decoded camera pixel format.");
      return false;
  }

  const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
  if (stride <= 0) {
    gst_video_frame_unmap(&frame);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Camera frame stride is invalid.");
    return false;
  }

  const auto width = static_cast<uint32_t>(GST_VIDEO_INFO_WIDTH(&video_info));
  const auto height = static_cast<uint32_t>(GST_VIDEO_INFO_HEIGHT(&video_info));
  const auto stamp = this->now();
  auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
  image_msg->header.stamp = stamp;
  image_msg->header.frame_id = camera_config_.frame_id;
  image_msg->width = width;
  image_msg->height = height;
  image_msg->encoding = encoding;
  image_msg->is_bigendian = false;
  image_msg->step = width * output_channel_count;
  image_msg->data.resize(image_msg->step * height);

  const auto* plane_data = reinterpret_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
  for (uint32_t row = 0; row < height; ++row) {
    const auto* src_row = plane_data + row * stride;
    auto* dst_row = image_msg->data.data() + row * image_msg->step;
    if (!drop_alpha_channel) {
      std::memcpy(dst_row, src_row, image_msg->step);
      continue;
    }

    for (uint32_t column = 0; column < width; ++column) {
      const auto src_index = column * input_channel_count;
      const auto dst_index = column * output_channel_count;
      dst_row[dst_index + 0] = src_row[src_index + 0];
      dst_row[dst_index + 1] = src_row[src_index + 1];
      dst_row[dst_index + 2] = src_row[src_index + 2];
    }
  }

  gst_video_frame_unmap(&frame);
  last_camera_publish_time_ = std::chrono::steady_clock::now();
  publishCameraInfo(width, height, stamp);
  if (has_image_subscribers) {
    cameraImagePublisher_->publish(std::move(image_msg));
  }
  return true;
}

void BlueROVBridge::publishCameraInfo(const uint32_t width, const uint32_t height, const rclcpp::Time& stamp) {
  if (cameraInfoPublisher_ == nullptr) {
    return;
  }

  if (cameraInfoPublisher_->get_subscription_count() == 0 &&
      cameraInfoPublisher_->get_intra_process_subscription_count() == 0) {
    return;
  }

  if (camera_config_.camera_info_publish_rate_hz > 0.0) {
    const auto now_steady = std::chrono::steady_clock::now();
    const auto min_period = std::chrono::duration<double>(1.0 / camera_config_.camera_info_publish_rate_hz);
    if (last_camera_info_publish_time_.time_since_epoch().count() != 0 &&
        now_steady - last_camera_info_publish_time_ < min_period) {
      return;
    }
    last_camera_info_publish_time_ = now_steady;
  }

  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header.stamp = stamp;
  info_msg.header.frame_id = camera_config_.frame_id;
  info_msg.width = width;
  info_msg.height = height;
  info_msg.distortion_model = "plumb_bob";
  info_msg.k = {1.0, 0.0, static_cast<double>(width) / 2.0,
                0.0, 1.0, static_cast<double>(height) / 2.0,
                0.0, 0.0, 1.0};
  info_msg.r = {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
  info_msg.p = {1.0, 0.0, static_cast<double>(width) / 2.0, 0.0,
                0.0, 1.0, static_cast<double>(height) / 2.0, 0.0,
                0.0, 0.0, 1.0, 0.0};
  cameraInfoPublisher_->publish(info_msg);
}

void BlueROVBridge::cameraStreamLoop() {
  bool use_hw_decoder = camera_config_.use_hw_decoder;

  while (!camera_stop_requested_.load()) {
    if (!createCameraPipeline(use_hw_decoder)) {
      if (use_hw_decoder) {
        RCLCPP_WARN(this->get_logger(), "Falling back to CPU H.264 decoding because the Jetson pipeline did not start.");
        use_hw_decoder = false;
        continue;
      }

      std::this_thread::sleep_for(kCameraRestartDelay);
      continue;
    }

    GstBus* bus = nullptr;
    GstAppSink* appsink = nullptr;
    {
      std::lock_guard<std::mutex> lock(camera_pipeline_mutex_);
      if (camera_pipeline_ != nullptr) {
        bus = gst_element_get_bus(camera_pipeline_);
      }
      if (camera_appsink_ != nullptr) {
        appsink = GST_APP_SINK(camera_appsink_);
      }
    }

    if (bus == nullptr || appsink == nullptr) {
      if (bus != nullptr) {
        gst_object_unref(bus);
      }
      destroyCameraPipeline();
      std::this_thread::sleep_for(kCameraRestartDelay);
      continue;
    }

    bool restart_pipeline = false;
    while (!camera_stop_requested_.load()) {
      GstMessage* message = gst_bus_timed_pop_filtered(
          bus,
          0,
          static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING));
      if (message != nullptr) {
        switch (GST_MESSAGE_TYPE(message)) {
          case GST_MESSAGE_ERROR: {
            GError* error = nullptr;
            gchar* debug = nullptr;
            gst_message_parse_error(message, &error, &debug);
            RCLCPP_ERROR(
                this->get_logger(),
                "Camera pipeline error from %s: %s",
                GST_OBJECT_NAME(message->src),
                error != nullptr ? error->message : "unknown error");
            if (error != nullptr) {
              g_error_free(error);
            }
            if (debug != nullptr) {
              g_free(debug);
            }
            restart_pipeline = true;
            break;
          }
          case GST_MESSAGE_EOS:
            RCLCPP_WARN(this->get_logger(), "Camera pipeline reached EOS. Restarting stream.");
            restart_pipeline = true;
            break;
          case GST_MESSAGE_WARNING: {
            GError* error = nullptr;
            gchar* debug = nullptr;
            gst_message_parse_warning(message, &error, &debug);
            RCLCPP_WARN(
                this->get_logger(),
                "Camera pipeline warning from %s: %s",
                GST_OBJECT_NAME(message->src),
                error != nullptr ? error->message : "unknown warning");
            if (error != nullptr) {
              g_error_free(error);
            }
            if (debug != nullptr) {
              g_free(debug);
            }
            break;
          }
          default:
            break;
        }
        gst_message_unref(message);
      }

      if (restart_pipeline) {
        break;
      }

      GstSample* sample = gst_app_sink_try_pull_sample(appsink, kCameraSampleTimeout.count() * GST_MSECOND);
      if (sample == nullptr) {
        continue;
      }

      publishCameraSample(sample);
      gst_sample_unref(sample);
    }

    gst_object_unref(bus);
    destroyCameraPipeline();

    if (!camera_stop_requested_.load() && restart_pipeline) {
      std::this_thread::sleep_for(kCameraRestartDelay);
    }
  }
}
#endif

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

  RCLCPP_INFO(this->get_logger(), "Listening for MAVLink on 0.0.0.0:%d", port_);

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

          case MAVLINK_MSG_ID_SCALED_IMU:
            handleScaledImu(msg);
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
            
          case MAVLINK_MSG_ID_BATTERY_STATUS:
            handleBatteryStatus(msg);
            break;

          case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            handleServoOutputRaw(msg);
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

void BlueROVBridge::sendMavlinkHeartbeat() {
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

void BlueROVBridge::configureDataStreams() {
  ++stream_config_attempts_;
  RCLCPP_INFO(this->get_logger(), "Configuring ArduSub data streams (attempt %d/3).",
              stream_config_attempts_);
  setMessageInterval(MAVLINK_MSG_ID_HEARTBEAT, 1.0f);
  setMessageInterval(MAVLINK_MSG_ID_SCALED_IMU, imu_rate_hz_);
  setMessageInterval(MAVLINK_MSG_ID_SCALED_PRESSURE2, pressure_rate_hz_);
  setMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0f);
  setMessageInterval(MAVLINK_MSG_ID_DISTANCE_SENSOR, 5.0f);
  setMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, 2.0f);
  setMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, servo_output_rate_hz_);
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
    case MAVLINK_MSG_ID_SCALED_IMU:
      return "SCALED_IMU";
    case MAVLINK_MSG_ID_SCALED_PRESSURE2:
      return "SCALED_PRESSURE2";
    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
      return "GPS_GLOBAL_ORIGIN";
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
      return "DISTANCE_SENSOR";
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

  if (!got_heartbeat_ && (hb.type == MAV_TYPE_SUBMARINE) ) {
    target_system_    = msg.sysid;
    target_component_ = msg.compid;
    got_heartbeat_    = true;

    last_heartbeat_time_ = this->now();

    // Reply to the exact UDP endpoint that supplied the autopilot heartbeat.
    // BlueOS udpout endpoints commonly use a dynamically selected source port,
    // so assuming that its source port matches our local listening port drops
    // message-interval requests and all subsequent commands.
    remote_addr_ = sender_addr;
    if (simulation_mode_){
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

    // Configure immediately, then retry from the watchdog after the UDP route
    // has had time to become fully bidirectional.
    stream_config_attempts_ = 0;
    configureDataStreams();
  } 
  else if (msg.sysid == target_system_ && msg.compid == target_component_) {
    last_heartbeat_time_ = this->now();
    return;
  }
}

void BlueROVBridge::ardusubHeartbeatWatchdog() {
  
  if (!got_heartbeat_) return;

  if (stream_config_attempts_ < 3) {
    configureDataStreams();
  }

  rclcpp::Duration elapsed = this->now() - last_heartbeat_time_;

  if (elapsed > HeartbeatTimeout) {
    RCLCPP_WARN(get_logger(), "Heartbeat timeout—autopilot disconnected/rebooted");
    got_heartbeat_ = false;
    target_system_ = 0;
    target_component_ = 0;
    stream_config_attempts_ = 0;
    last_heartbeat_time_ = rclcpp::Time(0,0,this->get_clock()->get_clock_type());
  }
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

  global_velocity_msg->linear.x = pos_int.vx / 100.0;  // cm/s → m/s
  global_velocity_msg->linear.y = pos_int.vy / 100.0;
  global_velocity_msg->linear.z = pos_int.vz / 100.0;
  globalVelocityActualPublisher_->publish(*global_velocity_msg);
}

void BlueROVBridge::handleAttitude(const mavlink_message_t& msg){
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(&msg, &attitude);
  
  global_velocity_msg->angular.x = attitude.rollspeed;          //Roll rate in rad/s
  global_velocity_msg->angular.y = attitude.pitchspeed;        //Pitch rate in rad/s
  global_velocity_msg->angular.z = attitude.yawspeed;           //Yaw rate in rad/s
  globalVelocityActualPublisher_->publish(*global_velocity_msg);
  
}

void BlueROVBridge::handleScaledImu(const mavlink_message_t& msg) {
  mavlink_scaled_imu_t scaled_imu;
  mavlink_msg_scaled_imu_decode(&msg, &scaled_imu);
  publishImuAndMag(
      scaled_imu.time_boot_ms, 1000000ULL,
      scaled_imu.xacc, scaled_imu.yacc, scaled_imu.zacc,
      scaled_imu.xgyro, scaled_imu.ygyro, scaled_imu.zgyro,
      scaled_imu.xmag, scaled_imu.ymag, scaled_imu.zmag);
}

void BlueROVBridge::publishImuAndMag(
    const uint64_t boot_time, const uint64_t nanoseconds_per_unit,
    const int16_t xacc, const int16_t yacc, const int16_t zacc,
    const int16_t xgyro, const int16_t ygyro, const int16_t zgyro,
    const int16_t xmag, const int16_t ymag, const int16_t zmag) {
  sensor_msgs::msg::Imu imu_msg;
  sensor_msgs::msg::MagneticField magnetic_field_msg;
  imu_msg.header.frame_id = imu_frame_id_;
  imu_msg.header.stamp = ardusubBootTimeToRos(boot_time, nanoseconds_per_unit);
  magnetic_field_msg.header = imu_msg.header;

  const Eigen::Vector3d acceleration = kFrdToFlu * Eigen::Vector3d(
      static_cast<double>(xacc) * kMilligToMps2,
      static_cast<double>(yacc) * kMilligToMps2,
      static_cast<double>(zacc) * kMilligToMps2);
  const Eigen::Vector3d angular_velocity = kFrdToFlu * Eigen::Vector3d(
      static_cast<double>(xgyro) * kMradpsToRadps,
      static_cast<double>(ygyro) * kMradpsToRadps,
      static_cast<double>(zgyro) * kMradpsToRadps);
  const Eigen::Vector3d magnetic_field = kFrdToFlu * Eigen::Vector3d(
      static_cast<double>(xmag) * kMilligaussToTesla,
      static_cast<double>(ymag) * kMilligaussToTesla,
      static_cast<double>(zmag) * kMilligaussToTesla);

  imu_msg.linear_acceleration.x = acceleration.x();
  imu_msg.linear_acceleration.y = acceleration.y();
  imu_msg.linear_acceleration.z = acceleration.z();
  imu_msg.angular_velocity.x = angular_velocity.x();
  imu_msg.angular_velocity.y = angular_velocity.y();
  imu_msg.angular_velocity.z = angular_velocity.z();
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = -1.0;
  imu_msg.angular_velocity_covariance.fill(-1.0);
  imu_msg.linear_acceleration_covariance.fill(-1.0);

  magnetic_field_msg.magnetic_field.x = magnetic_field.x();
  magnetic_field_msg.magnetic_field.y = magnetic_field.y();
  magnetic_field_msg.magnetic_field.z = magnetic_field.z();
  magnetic_field_msg.magnetic_field_covariance[0] = -1.0;

  imuPublisher_->publish(imu_msg);
  magneticFieldPublisher_->publish(magnetic_field_msg);
}

void BlueROVBridge::handleCommandAck(const mavlink_message_t& msg) {
  mavlink_command_ack_t ack;
  mavlink_msg_command_ack_decode(&msg, &ack);
  if (ack.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
    RCLCPP_INFO(this->get_logger(),
                "ArduSub acknowledged SET_MESSAGE_INTERVAL with MAV_RESULT=%u.",
                static_cast<unsigned>(ack.result));
  }
}

void BlueROVBridge::handleScaledPressure2(const mavlink_message_t& msg){
  mavlink_scaled_pressure2_t pressure2;
  mavlink_msg_scaled_pressure2_decode(&msg, &pressure2);

  sensor_msgs::msg::FluidPressure pressure_msg;
  // SCALED_PRESSURE2 carries ArduSub boot time in milliseconds.
  pressure_msg.header.stamp = ardusubBootTimeToRos(pressure2.time_boot_ms, 1000000ULL);
  pressure_msg.header.frame_id = pressure_frame_id_;
  pressure_msg.fluid_pressure = static_cast<double>(pressure2.press_abs) * 100.0;  // hPa -> Pa
  pressure_msg.variance = 0.0;

  pressureScaled2Publisher_->publish(pressure_msg);
}

rclcpp::Time BlueROVBridge::ardusubBootTimeToRos(
    const uint64_t boot_time,
    const uint64_t nanoseconds_per_unit) {
  const uint64_t boot_time_ns = boot_time * nanoseconds_per_unit;
  const int64_t now_ns = this->now().nanoseconds();
  const int64_t observed_offset_ns = now_ns - static_cast<int64_t>(boot_time_ns);

  // Establish the offset on the first sensor sample. Reset it if ArduSub
  // reboots and its monotonic boot clock moves backwards.
  constexpr uint64_t kRebootBackwardsJumpNs = 5000000000ULL;
  if (!ardusub_boot_to_ros_offset_ns_.has_value() ||
      boot_time_ns + kRebootBackwardsJumpNs < last_ardusub_boot_time_ns_) {
    ardusub_boot_to_ros_offset_ns_ = observed_offset_ns;
  } else if (observed_offset_ns < *ardusub_boot_to_ros_offset_ns_) {
    // The first MAVLink packet can be stale data queued while the UDP link is
    // coming up. Refine the offset using the lowest observed transport delay;
    // otherwise subsequent live samples can be stamped seconds in the future.
    ardusub_boot_to_ros_offset_ns_ = observed_offset_ns;
  }
  last_ardusub_boot_time_ns_ = std::max(last_ardusub_boot_time_ns_, boot_time_ns);

  return rclcpp::Time(
      *ardusub_boot_to_ros_offset_ns_ + static_cast<int64_t>(boot_time_ns),
      this->get_clock()->get_clock_type());
}
