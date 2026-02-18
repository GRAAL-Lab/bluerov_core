#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS 2 message headers
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/string.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/msg/heart_beat.hpp"                           
#include "std_msgs/msg/bool.hpp"                     
#include "std_msgs/msg/float64.hpp"                  
#include "std_msgs/msg/int8.hpp"                     
#include "std_msgs/msg/int32.hpp"     
#include "auv_core_helper/msg/battery_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "auv_core_helper/msg/gimbal_status.hpp"

// ROS 2 service headers
#include "std_srvs/srv/set_bool.hpp"             
#include "auv_core_helper/srv/set_flight_mode.hpp" 
#include "auv_core_helper/srv/set_global_origin.hpp"
#include "auv_core_helper/srv/set_gimbal_attitude.hpp"

// AUV-specific topic names
#include "auv_core_helper/topicnames.hpp"
#include "auv_core_helper/bridgemode.hpp"
#include <Eigen/Dense>

// Include the MAVLink C headers
extern "C" {
    #include <mavlink/v2.0/common/mavlink.h>
}

// Standard libraries for sockets (UDP example)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <rmw/rmw.h>
#include <optional>
#include <limits>
#include <array>
#include <vector>

using SetBoolSrv = std_srvs::srv::SetBool;
using SetModeSrv = auv_core_helper::srv::SetFlightMode;
using SetGimbalAttitudeSrv = auv_core_helper::srv::SetGimbalAttitude;

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
    rclcpp::Publisher<auv_core_helper::msg::HeartBeat>::SharedPtr ardusubHeartBeatPublisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr bridgeHeartBeatPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr globalOriginPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::BatteryStatus>::SharedPtr batteryStatusPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr globalPoseActualPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr globalVelocityActualPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dvlDistancePublisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ekfStatusPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::GimbalStatus>::SharedPtr gimbalStatusPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr forcesDesiredPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr forcesActualPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressureScaled2Publisher_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySwitchSubscription_;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr globalPoseDesiredSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr globalVelocityDesiredSubscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desiredCtrlModeSubscription_;

    /* How long we're willing to wait before we fail the request */
    static const rclcpp::Duration kSrvTimeout;      ///< watchdog for deferred services
    static const rclcpp::Duration HeartbeatTimeout; ///< watchdog for autopilot heartbeat
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr armingService_;
    rclcpp::Service<auv_core_helper::srv::SetFlightMode>::SharedPtr flightModeService_;
    rclcpp::Service<auv_core_helper::srv::SetGlobalOrigin>::SharedPtr setGlobalOriginService_;   
    rclcpp::Service<auv_core_helper::srv::SetGimbalAttitude>::SharedPtr gimbalService_;

    //--------------------------------------------------------------------------
    // Timers
    //--------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr bridge_heartbeat_timer_; 
    rclcpp::TimerBase::SharedPtr ardusub_heartbeat_watchdog_timer_;
    rclcpp::TimerBase::SharedPtr data_timer_;         // Timer for MAVLink data reception
    rclcpp::TimerBase::SharedPtr exec_timer_;         // Timer for execution loop

    //--------------------------------------------------------------------------
    // MAVLink Socket / Connection
    //--------------------------------------------------------------------------
    int sock_fd_{-1};                   // UDP socket file descriptor
    struct sockaddr_in remote_addr_{};  // Remote address for sending MAVLink
    int port_{} ;
    std::string remote_addr_str_ = "";

    // Our GCS system ID
    uint8_t system_id_{0};            // MAVLink system ID (255 = ground station)
    uint8_t component_id_{0};         // MAVLink component ID

    // Autopilot IDs (discovered from heartbeat)
    uint8_t target_system_{0};          // Target system ID (from heartbeat)
    uint8_t target_component_{0};       // Target component ID (from heartbeat)
    bool got_heartbeat_{false};         // Flag indicating if heartbeat was received
    rclcpp::Time last_heartbeat_time_;   // Timestamp of last heartbeat


    //--------------------------------------------------------------------------
    //Declarations
    //--------------------------------------------------------------------------
    bool simulation_mode_;
    std::string imu_topic_{auv_core_helper::topicnames::imu_data_raw};
    std::string imu_frame_id_{"base_link"};
    float imu_rate_hz_{100.0f};
    float servo_output_rate_hz_{50.0f};
    std::string forces_desired_topic_{"/auv/forces_desired"};
    std::string forces_actual_topic_{auv_core_helper::topicnames::forces_actual};
    std::string pressure_topic_{auv_core_helper::topicnames::pressure_scaled_2};
    std::string pressure_frame_id_{"base_link"};
    std::string thrust_table_csv_path_;
    float pressure_rate_hz_{20.0f};
        Eigen::Matrix3d imu_rotation_matrix_ = (Eigen::Matrix3d() <<
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0).finished();

    bool failsafe_active_{false};
    
    mavlink_heartbeat_t hb;
    mavlink_command_ack_t ack;
    mavlink_set_position_target_global_int_t position_target_global_;
    mavlink_command_long_t condition_yaw_ ;
    mavlink_gps_global_origin_t gps_global_origin;
    
    std::unique_ptr<auv_core_helper::msg::PoseStamped> global_pose_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
    std::unique_ptr<geometry_msgs::msg::Twist> global_velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();
    double latest_battery_voltage_v_{std::numeric_limits<double>::quiet_NaN()};
    static constexpr size_t kThrusterCount{8};
    static constexpr size_t kVoltageBinCount{3};
    std::array<double, kVoltageBinCount> thrust_voltage_bins_v_{{14.0, 16.0, 18.0}};
    std::vector<double> thrust_pwm_values_;
    std::vector<std::vector<double>> thrust_table_newtons_;
    bool thrust_table_loaded_{false};
    sensor_msgs::msg::JointState forces_actual_msg_;

    Eigen::VectorXd poseGoalGlobal = Eigen::VectorXd(6); ///< Desired pose goal in global coordinates.
    Eigen::VectorXd poseGoalGlobalLast = Eigen::VectorXd(6); ///< Last desired pose goal in global coordinates.
    Eigen::VectorXd velocityGoalGlobal = Eigen::VectorXd(6); ///< Desired linear and angular velocities in global coordinates.
    Eigen::VectorXd velGoalGlobalLast = Eigen::VectorXd(6); ///< Last desired velocity goal in global coordinates.

    bool poseGoalGlobalChanged = false; ///< Flag indicating if the pose goal has changed.
    const double LAT_LON_EPS{1e-6};   // ≈11 cm
    const double DEPTH_EPS{0.02};   // 2 cm
    const double YAW_EPS{0.01};   // ≈0.6°

    bool velGoalGlobalChanged = false; ///< Flag indicating if the velocity goal has changed.
    const double VELX_EPS{0.01}; ///< Velocity X tolerance
    const double VELY_EPS{0.01}; ///< Velocity Y tolerance
    const double VELZ_EPS{0.01}; ///< Velocity Z tolerance
    const double ANGX_EPS{0.01}; ///< Angular X tolerance
    const double ANGY_EPS{0.01}; ///< Angular Y tolerance
    const double ANGZ_EPS{0.01}; ///< Angular Z tolerance

    std::string ctrlMode = "NOT_SET"; ///< Current flight mode, default is PoseCtrl.

    struct PendingArm {
    std::shared_ptr<rmw_request_id_t>                 header;
    std::shared_ptr<std_srvs::srv::SetBool::Response> resp;
    bool                                             want_arm;
    rclcpp::Time                                     deadline;
    };
    std::optional<PendingArm>  pending_arm_;
    
    struct PendingMode {
    std::shared_ptr<rmw_request_id_t>                              header;
    std::shared_ptr<auv_core_helper::srv::SetFlightMode::Response> resp;
    int32_t                                                        desired_custom;
    rclcpp::Time                                                   deadline;
    };
    std::optional<PendingMode> pending_mode_;
    
    struct PendingGimbalAttitude {
    std::shared_ptr<rmw_request_id_t>                                  header;
    std::shared_ptr<auv_core_helper::srv::SetGimbalAttitude::Response> resp;
    std::shared_ptr<auv_core_helper::srv::SetGimbalAttitude::Request>  request;
    rclcpp::Time                                                       deadline;
    };
    std::optional<PendingGimbalAttitude> pending_gimbal_attitude_;

    //--------------------------------------------------------------------------
    // Internal Methods
    //--------------------------------------------------------------------------
    void initMavlinkConnection();

    void receiveData();

    void sendMavlinkMessage(const mavlink_message_t& msg);

    void bridgeHeartbeat();  
    
    void ardusubHeartbeatWatchdog();

    void setMessageInterval(uint16_t message_id, float frequency_hz);

    const char* get_message_name(uint16_t message_id);    
    
    void handleArduSubHeartbeat(const mavlink_message_t& msg, const sockaddr_in& sender_addr);

    void handleGlobalOrigin(const mavlink_message_t& msg);

    void handleBatteryStatus(const mavlink_message_t& msg);

    void handleEkfStatus(const mavlink_message_t& msg);

    void handleGlobalPositionInt(const mavlink_message_t& msg);

    void handleGlobalVelocityInt(const mavlink_message_t& msg);

    void handleAttitude(const mavlink_message_t& msg);

    void handleRawImu(const mavlink_message_t& msg);

    void handleScaledPressure2(const mavlink_message_t& msg);

    void handleDvlDistance(const mavlink_message_t& msg);

    void handleServoOutputRaw(const mavlink_message_t& msg);
     
    void handleCommandAck(const mavlink_message_t& msg);

    void handleGimbalStatus(const mavlink_message_t& msg);
    
    void safetySwitchCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void armingServiceCallback(
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<SetBoolSrv::Request> request);

    void setArmState(bool arm_vehicle);    

    void flightModeServiceCallback(
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<SetModeSrv::Request> request);

    int32_t mapModeStringToNumber(const std::string & mode) const;    

    void setFlightMode(const std::string& mode);

    void setGlobalOriginServiceCallback(
        const std::shared_ptr<auv_core_helper::srv::SetGlobalOrigin::Request> request,
        std::shared_ptr<auv_core_helper::srv::SetGlobalOrigin::Response> response);

    void setGlobalOrigin(mavlink_set_gps_global_origin_t& set_gps_global_origin);  
    
    void gimbalServiceCallback(
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<auv_core_helper::srv::SetGimbalAttitude::Request> request);

    void setGimbalAttitude(float gimbal_pitch, float gimbal_yaw); 
    
    void rcChannelsOverride(uint16_t rc[]); 

    void setServo(uint8_t servoID,uint16_t pwm);

    void cycleServo(uint8_t servoID,uint16_t pwm,uint16_t cycleCount,uint16_t cycleTime);

    void desiredCtrlModeCallback(const std_msgs::msg::String::SharedPtr msg);
    
    void globalPoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);

    void globalVelocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool loadThrustTableFromCsv(const std::string& csv_path);

    double getThrust(double pwm, double voltage) const;

    void Execute();
    
    void SetPositionTargetGlobalInt(const mavlink_set_position_target_global_int_t& position_target_global_);

    void sendConditionYaw(const mavlink_command_long_t& condition_yaw_);                                
};
