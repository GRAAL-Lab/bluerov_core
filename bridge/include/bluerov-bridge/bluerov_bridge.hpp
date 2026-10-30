#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS 2 message headers
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#ifdef BRIDGE_ENABLE_CAMERA
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#endif
#include "std_msgs/msg/float64.hpp"                  
#include "std_srvs/srv/set_bool.hpp"
#include "helper_lib.hpp"
#include "bleurov_core_helper/srv/set_flight_mode.hpp"
#include "topicnames.hpp"
#include <Eigen/Dense>

// Include the MAVLink C headers
extern "C" {
    #include <mavlink/v2.0/common/mavlink.h>
}

// Standard libraries for sockets (UDP example)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <optional>
#include <limits>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#ifdef BRIDGE_ENABLE_CAMERA
typedef struct _GstElement GstElement;
typedef struct _GstSample GstSample;
#endif

class BlueROVBridge : public rclcpp::Node
{
public:
    /// Constructor
    BlueROVBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /// Destructor
    ~BlueROVBridge();

private:
    //--------------------------------------------------------------------------
    // ROS Publishers, Subscribers & Services
    //--------------------------------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr globalVelocityActualPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dvlDistancePublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magneticFieldPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr forcesActualPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressureScaled2Publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocityDesiredSubscription_;
#ifdef BRIDGE_ENABLE_CAMERA
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cameraImagePublisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPublisher_;
#endif

    using SetArm = std_srvs::srv::SetBool;
    using SetFlightMode = bleurov_core_helper::srv::SetFlightMode;
    rclcpp::Service<SetArm>::SharedPtr armingService_;
    rclcpp::Service<SetFlightMode>::SharedPtr flightModeService_;
    
    static const rclcpp::Duration HeartbeatTimeout; ///< watchdog for autopilot heartbeat

    //--------------------------------------------------------------------------
    // Timers
    //--------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr mavlink_heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr ardusub_heartbeat_watchdog_timer_;
    rclcpp::TimerBase::SharedPtr data_timer_;         // Timer for MAVLink data reception
    rclcpp::TimerBase::SharedPtr rc_override_timer_;

    //--------------------------------------------------------------------------
    // MAVLink Socket / Connection
    //--------------------------------------------------------------------------
    int sock_fd_{-1};                   // UDP socket file descriptor
    struct sockaddr_in remote_addr_{};  // Remote address for sending MAVLink
    int port_{14550};
    std::string remote_addr_str_{"192.168.2.2"};

    // Our GCS system ID
    uint8_t system_id_{42};            // MAVLink system ID (255 = ground station)
    uint8_t component_id_{190};         // MAVLink component ID

    // Autopilot IDs (discovered from heartbeat)
    uint8_t target_system_{0};          // Target system ID (from heartbeat)
    uint8_t target_component_{0};       // Target component ID (from heartbeat)
    bool got_heartbeat_{false};         // Flag indicating if heartbeat was received
    rclcpp::Time last_heartbeat_time_;   // Timestamp of last heartbeat
    int stream_config_attempts_{0};


    //--------------------------------------------------------------------------
    //Declarations
    //--------------------------------------------------------------------------
    bool simulation_mode_{false};
    std::string velocity_actual_topic_{"/auv/global/velocity_actual"};
    std::string dvl_distance_topic_{"/auv/dvl/distance"};
    std::string imu_topic_{"/auv/imu/data_raw"};
    std::string magnetic_field_topic_{"/auv/imu/magnetic_field"};
    std::string imu_frame_id_{"auv_base_link"};
    float imu_rate_hz_{200.0f};
    float servo_output_rate_hz_{50.0f};
    std::string forces_actual_topic_{"/auv/forces_actual"};
    std::string pressure_topic_{"/auv/pressure/scaled2"};
    std::string pressure_frame_id_{"auv_base_link"};
    std::string thrust_table_csv_path_;
    float pressure_rate_hz_{100.0f};
    bool rc_override_enabled_{true};
    bool rc_override_require_poshold_{true};
    std::string velocity_desired_topic_{bleurov_core_helper::topicnames::velocity_desired};
    std::string velocity_desired_frame_id_{"auv_base_link"};
    double rc_override_rate_hz_{100.0};
    double velocity_command_timeout_s_{0.25};
    // KCL command order: surge, sway, heave, roll rate, pitch rate, yaw rate.
    // Linear values use m/s and angular values use rad/s in ROS FLU.
    std::array<double, 6> velocity_command_min_flu_{{-2.0, -1.8, -0.55, -2.0, -2.2, -1.85}};
    std::array<double, 6> velocity_command_max_flu_{{ 2.0,  1.8,  0.55,  2.0,  2.2,  1.85}};
    // ArduSub RCMAP order: roll, pitch, throttle, yaw, forward, lateral.
    std::array<uint8_t, 6> rc_override_channels_{{1, 2, 3, 4, 6, 7}};
    // PWM order: forward, lateral/right, vertical/up, yaw/clockwise.
    std::array<uint16_t, 4> rc_override_pwm_min_{{1100, 1100, 1100, 1100}};
    std::array<uint16_t, 4> rc_override_pwm_neutral_{{1500, 1500, 1500, 1500}};
    std::array<uint16_t, 4> rc_override_pwm_max_{{1900, 1900, 1900, 1900}};
    uint16_t rc_override_attitude_neutral_pwm_{1500};
    std::mutex velocity_command_mutex_;
    geometry_msgs::msg::Twist latest_velocity_command_frd_;
    std::chrono::steady_clock::time_point latest_velocity_command_received_{};
    bool velocity_command_received_{false};
    bool rc_override_active_{false};
    std::optional<int64_t> ardusub_boot_to_ros_offset_ns_;
    uint64_t last_ardusub_boot_time_ns_{0};
    struct CameraStreamConfig {
        // Camera streaming is opt-in.  Keeping this false guarantees that a
        // plain `ros2 run bridge bridge_node` does not create image topics or
        // try to open a video source.
        bool enabled{false};
        std::string source_uri{"udp://192.168.2.3:5600"};
        std::string rtp_caps{"application/x-rtp, media=video, encoding-name=H264, payload=96, clock-rate=90000"};
        std::string topic{"/auv/camera/image_raw"};
        std::string info_topic{"/auv/camera/camera_info"};
        std::string frame_id{"camera_optical_frame"};
        bool use_hw_decoder{true};
        bool qos_reliable{false};
        bool enable_max_performance{true};
        int preview_width{960};
        int preview_height{540};
        double preview_max_fps{15.0};
        std::string output_encoding{"rgba8"};
        double camera_info_publish_rate_hz{2.0};
        int rtp_latency_ms{75};
        int udp_buffer_size_bytes{2097152};
        int appsink_max_buffers{1};
        bool appsink_drop{true};
        std::string pipeline_override{};
    } camera_config_;
#ifdef BRIDGE_ENABLE_CAMERA
    std::thread camera_thread_;
    std::atomic<bool> camera_stop_requested_{false};
    std::mutex camera_pipeline_mutex_;
    GstElement* camera_pipeline_{nullptr};
    GstElement* camera_appsink_{nullptr};
    std::chrono::steady_clock::time_point last_camera_publish_time_{};
    std::chrono::steady_clock::time_point last_camera_info_publish_time_{};
    std::optional<uint64_t> camera_media_time_origin_ns_;
    std::optional<rclcpp::Time> camera_ros_time_origin_;
#endif
    mavlink_heartbeat_t hb;
    std::unique_ptr<geometry_msgs::msg::Twist> global_velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();
    double latest_battery_voltage_v_{std::numeric_limits<double>::quiet_NaN()};
    std::optional<std::chrono::steady_clock::time_point> battery_voltage_wait_start_;
    static constexpr size_t kThrusterCount{8};
    static constexpr size_t kVoltageBinCount{3};
    std::array<double, kVoltageBinCount> thrust_voltage_bins_v_{{14.0, 16.0, 18.0}};
    std::vector<double> thrust_pwm_values_;
    std::vector<std::vector<double>> thrust_table_newtons_;
    bool thrust_table_loaded_{false};
    sensor_msgs::msg::JointState forces_actual_msg_;


    //--------------------------------------------------------------------------
    // Internal Methods
    //--------------------------------------------------------------------------
    void initMavlinkConnection();

    void receiveData();

    void sendMavlinkMessage(const mavlink_message_t& msg);

    void sendMavlinkHeartbeat();

    void ardusubHeartbeatWatchdog();

    void setMessageInterval(uint16_t message_id, float frequency_hz);

    void configureDataStreams();

    const char* get_message_name(uint16_t message_id);    
    
    void handleArduSubHeartbeat(const mavlink_message_t& msg, const sockaddr_in& sender_addr);

    void handleBatteryStatus(const mavlink_message_t& msg);

    void handleGlobalPositionInt(const mavlink_message_t& msg);

    void handleGlobalVelocityInt(const mavlink_message_t& msg);

    void handleAttitude(const mavlink_message_t& msg);

    void handleScaledImu(const mavlink_message_t& msg);

    void publishImuAndMag(uint64_t boot_time, uint64_t nanoseconds_per_unit,
                          int16_t xacc, int16_t yacc, int16_t zacc,
                          int16_t xgyro, int16_t ygyro, int16_t zgyro,
                          int16_t xmag, int16_t ymag, int16_t zmag);

    void handleCommandAck(const mavlink_message_t& msg);

    void armingServiceCallback(
        const std::shared_ptr<SetArm::Request> request,
        std::shared_ptr<SetArm::Response> response);

    void flightModeServiceCallback(
        const std::shared_ptr<SetFlightMode::Request> request,
        std::shared_ptr<SetFlightMode::Response> response);

    bool sendArmCommand(bool arm_vehicle);

    std::optional<uint32_t> flightModeNumber(const std::string& mode) const;

    bool sendFlightModeCommand(uint32_t custom_mode);

    void velocityDesiredCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void sendVelocityRcOverride();

    void releaseRcOverrides();

    void sendRcChannels(const std::array<uint16_t, 18>& channels);

    uint16_t velocityToPwm(
        double value, double input_min, double input_max, size_t pwm_axis) const;

    void handleScaledPressure2(const mavlink_message_t& msg);

    void handleDvlDistance(const mavlink_message_t& msg);

    void handleServoOutputRaw(const mavlink_message_t& msg);
     
    bool loadThrustTableFromCsv(const std::string& csv_path);

    double getThrust(double pwm, double voltage) const;

    rclcpp::Time ardusubBootTimeToRos(uint64_t boot_time, uint64_t nanoseconds_per_unit);

#ifdef BRIDGE_ENABLE_CAMERA
    void startCameraStream();

    void stopCameraStream();

    void cameraStreamLoop();

    bool createCameraPipeline(bool use_hw_decoder);

    void destroyCameraPipeline();

    std::string buildCameraPipelineDescription(bool use_hw_decoder) const;

    bool publishCameraSample(GstSample* sample);

    rclcpp::Time cameraMediaStamp(GstSample* sample);

    void publishCameraInfo(uint32_t width, uint32_t height, const rclcpp::Time& stamp);
#endif

};
