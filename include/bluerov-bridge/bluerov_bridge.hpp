#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS 2 message headers
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/msg/heart_beat.hpp"                           
#include "std_msgs/msg/bool.hpp"                     
#include "std_msgs/msg/float64.hpp"                  
#include "std_msgs/msg/int8.hpp"                     
#include "std_msgs/msg/int32.hpp"     
#include "auv_core_helper/msg/battery_status.hpp"
#include "tf2/LinearMath/Quaternion.h"


// ROS 2 service headers
#include "std_srvs/srv/set_bool.hpp"             
#include "auv_core_helper/srv/set_flight_mode.hpp" 
#include "auv_core_helper/srv/set_global_origin.hpp"

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

using SetBoolSrv = std_srvs::srv::SetBool;
using SetModeSrv = auv_core_helper::srv::SetFlightMode;

/**
 * @brief A ROS 2 node that interfaces with ArduPilot/BlueROV using MAVLink protocol.
 */
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
    rclcpp::Publisher<auv_core_helper::msg::HeartBeat>::SharedPtr heartBeatPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr globalOriginPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::BatteryStatus>::SharedPtr batteryStatusPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr globalPoseActualPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr globalVelocityActualPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dvlDistancePublisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ekfStatusPublisher_;
    
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr globalPoseDesiredSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr globalVelocityDesiredSubscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desiredCtrlModeSubscription_;

    /* How long we're willing to wait before we fail the request */
    static const rclcpp::Duration kSrvTimeout;      ///< watchdog for deferred services
    static const rclcpp::Duration HeartbeatTimeout; ///< watchdog for autopilot heartbeat
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr armingService_;
    rclcpp::Service<auv_core_helper::srv::SetFlightMode>::SharedPtr flightModeService_;
    rclcpp::Service<auv_core_helper::srv::SetGlobalOrigin>::SharedPtr setGlobalOriginService_;   

    //--------------------------------------------------------------------------
    // Timers
    //--------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr system_heartbeat_timer_; 
    rclcpp::TimerBase::SharedPtr autopilot_heartbeat_watchdog_timer_;
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
    uint64_t last_heartbeat_time_{0};   // Timestamp of last heartbeat


    //--------------------------------------------------------------------------
    //Declarations
    //--------------------------------------------------------------------------
    bool simulation_mode_;
    
    mavlink_heartbeat_t hb;
    mavlink_command_ack_t ack;
    mavlink_set_position_target_global_int_t position_target_global_;
    mavlink_command_long_t condition_yaw_ ;
    mavlink_gps_global_origin_t gps_global_origin;
    
    std::unique_ptr<auv_core_helper::msg::PoseStamped> global_pose_msg = std::make_unique<auv_core_helper::msg::PoseStamped>();
    std::unique_ptr<geometry_msgs::msg::Twist> global_velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();

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

    //--------------------------------------------------------------------------
    // Internal Methods
    //--------------------------------------------------------------------------
    /**
     * @brief Initialize MAVLink UDP socket connection
     */
    void initMavlinkConnection();
    
    /**
     * @brief Receive and process MAVLink data
     */
    void receiveData();

    /**
     * @brief Send a MAVLink message to ArduSub
     */
    void sendMavlinkMessage(const mavlink_message_t& msg);

    /**
     * @brief Send a MAVLink heartbeat message to ArduSub
     */
    void systemHeartbeat();  
    
    /**
     * @brief Watchdog for autopilot heartbeat
     */
     void autopilotHeartbeatWatchdog();
    
    /**
     * @brief Set the update interval for MAVLink messages
     */
    void setMessageInterval(uint16_t message_id, float frequency_hz);

    /**
   * @brief Get the string name of a MAVLink message from its ID.
   * 
   * @param message_id The ID of the MAVLink message.
   * @return const char* The name of the message, or "Unknown".
   */
    const char* get_message_name(uint16_t message_id);    
    
    /**
     * @brief Handle HEARTBEAT message
     * @param msg The received MAVLink message
     * @param sender_addr The sender's address
     */
    void handleHeartbeat(const mavlink_message_t& msg, const sockaddr_in& sender_addr);

    /**
     * @brief Handle GPS_GLOBAL_ORIGIN message
     * @param msg The received MAVLink message
     */
    void handleGlobalOrigin(const mavlink_message_t& msg);

    /**
     * @brief Handle BATTERY_STATUS message
     * @param msg The received MAVLink message
     */
    void handleBatteryStatus(const mavlink_message_t& msg);

    /**
     * @brief Handle EKF_STATUS message
     * @param msg The received MAVLink message
     */
    void handleEkfStatus(const mavlink_message_t& msg);
    
    /**
     * @brief Handle GLOBAL_POSITION_INT message
     * @param msg The received MAVLink message
     */
    void handleGlobalPositionInt(const mavlink_message_t& msg);

    /**
     * @brief Handle GLOBAL_POSITION_INT message
     * @param msg The received MAVLink message
     */
    void handleGlobalVelocityInt(const mavlink_message_t& msg);

    /**
     * @brief Handle ATTITUDE message
     * @param msg The received MAVLink message
     */
    void handleAttitude(const mavlink_message_t& msg);

    /**
    * @brief Handle DVL_DISTANCE message
    * @param msg The received MAVLink message
    */
    void handleDvlDistance(const mavlink_message_t& msg);
     
    /**
     * @brief Handle COMMAND_ACK message
     * @param msg The received MAVLink message
     */
    void handleCommandAck(const mavlink_message_t& msg);

    /**
     * @brief Service callback for setting the global origin.
     * @param request Service request containing the desired global origin.
     * @param response Service response indicating success/failure.
     */
    void setGlobalOriginServiceCallback(
        const std::shared_ptr<auv_core_helper::srv::SetGlobalOrigin::Request> request,
        std::shared_ptr<auv_core_helper::srv::SetGlobalOrigin::Response> response);

    /**
     * @brief Set the global origin via MAVLink
     * @param set_gps_global_origin The MAVLink message containing global origin data
     */
    void setGlobalOrigin(mavlink_set_gps_global_origin_t& set_gps_global_origin);

    /**
     * @brief Service callback for arming/disarming the vehicle.
     * @param request Service request containing boolean for arm (true) or disarm (false).
     * @param response Service response indicating success/failure.
     */
    void armingServiceCallback(
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<SetBoolSrv::Request> request);

    /**
     * @brief Set the arm state
     * @param arm The desired arm state
     */
    void setArmState(bool arm_vehicle);    

    /**
     * @brief Service callback for setting the vehicle's flight mode.
     * @param request Service request containing the desired flight mode string.
     * @param response Service response indicating success/failure.
     */
    void flightModeServiceCallback(
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<SetModeSrv::Request> request);

    /**
     * @brief Set the flight mode
     * @param mode The desired flight mode
     */
    void setFlightMode(const std::string& mode);
    
    /**
     * @brief Callback for receiving desired global pose
     * @param msg The received PoseStamped message for global coordinates
     */
    void globalPoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Callback for receiving desired global velocity
     * @param msg The received Twist message for global coordinates
     */
    void globalVelocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief Send a waypoint in global coordinates
     * 
     * Sends waypoint using SET_POSITION_TARGET_GLOBAL_INT message to enable
     * global positioning and navigation.
     * 
     */
    void SetPositionTargetGlobalInt(const mavlink_set_position_target_global_int_t& position_target_global_);

    /**
     * @brief Send MAV_CMD_CONDITION_YAW command to set vehicle heading
     * 
     */
    void sendConditionYaw(const mavlink_command_long_t& condition_yaw_);                            

    /**
     * @brief Main loop for processing MAVLink messages and ROS callbacks
     */
    void Execute();

    /**
     * @brief Callback for desired control mode
     * @param msg The received String message indicating the desired control mode
     */
    void desiredCtrlModeCallback(const std_msgs::msg::String::SharedPtr msg);


    /* ----------  PENDING-ARM  ---------- */
    struct PendingArm {
    std::shared_ptr<rmw_request_id_t>            header;
    std::shared_ptr<std_srvs::srv::SetBool::Response> resp;
    bool                                         want_arm;
    rclcpp::Time                                 deadline;
    };
    std::optional<PendingArm>  pending_arm_;

    /* ----------  PENDING-MODE  ---------- */
    struct PendingMode {
    std::shared_ptr<rmw_request_id_t>                  header;
    std::shared_ptr<auv_core_helper::srv::SetFlightMode::Response> resp;
    int32_t                                    desired_custom;
    rclcpp::Time                               deadline;
    };
    std::optional<PendingMode> pending_mode_;

    int32_t mapModeStringToNumber(const std::string & mode) const;
};
