#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS 2 message headers
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/msg/heart_beat.hpp"                           
#include "auv_core_helper/msg/rc_channels.hpp"
#include "std_msgs/msg/bool.hpp"                     
#include "std_msgs/msg/float64.hpp"                  
#include "std_msgs/msg/int8.hpp"                     
#include "std_msgs/msg/int32.hpp"                                   

// ROS 2 service headers
#include "std_srvs/srv/set_bool.hpp"             
#include "auv_core_helper/srv/set_flight_mode.hpp" 

// AUV-specific topic names
#include "auv_core_helper/topicnames.hpp"

// We will use Eigen for the NED->ENU transform
#include <Eigen/Dense>

// Include the MAVLink C headers
extern "C" {
    #include <mavlink/v2.0/common/mavlink.h>
}

// Standard libraries for sockets (UDP example)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

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
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr localPoseActualPublisher_;
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr globalPoseActualPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr localVelocityActualPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr globalVelocityActualPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dvlDistancePublisher_;
    
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr localPoseDesiredSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr localVelocityDesiredSubscription_;
    rclcpp::Subscription<auv_core_helper::msg::RCChannels>::SharedPtr rcChannelValuesDesiredSubscription_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr armingService_;
    rclcpp::Service<auv_core_helper::srv::SetFlightMode>::SharedPtr flightModeService_;
    
    //--------------------------------------------------------------------------
    // Timers
    //--------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr data_timer_;         // Timer for MAVLink data reception

    //--------------------------------------------------------------------------
    // MAVLink Socket / Connection
    //--------------------------------------------------------------------------
    int sock_fd_{-1};                   // UDP socket file descriptor
    struct sockaddr_in remote_addr_{};  // Remote address for sending MAVLink

    // Our GCS system ID
    uint8_t system_id_{255};            // MAVLink system ID (255 = ground station)
    uint8_t component_id_{190};         // MAVLink component ID

    // Autopilot IDs (discovered from heartbeat)
    uint8_t target_system_{0};          // Target system ID (from heartbeat)
    uint8_t target_component_{0};       // Target component ID (from heartbeat)
    bool got_heartbeat_{false};         // Flag indicating if heartbeat was received

    //=============================================================================
    // MAVLink declarations
    //=============================================================================
    mavlink_heartbeat_t hb;
    mavlink_command_ack_t ack;
    mavlink_set_position_target_local_ned_t position_target_;
    mavlink_set_position_target_global_int_t position_target_global_;
    mavlink_set_attitude_target_t attitude_target_;

    const uint16_t MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_POSITION = 0b00000000000000000000000000000001;
    const uint16_t MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_VELOCITY = 0b00000000000000000000000000000010;
    const uint16_t MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_YAW = 0b00000000000000000000000000000100;
    const uint16_t MAVLINK_POSITION_TARGET_LOCAL_NED_TYPE_MASK_YAW_RATE = 0b00000000000000000000000000001000;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_YAW_ANGLE = 0b00000000000000000000000000000001;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_YAW_RATE = 0b00000000000000000000000000000010;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_BODY_RATE_OUTPUT = 0b00000000000000000000000000000100;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_THRUST = 0b00000000000000000000000000001000;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_FORCE = 0b00000000000000000000000000001000;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_ANGULAR_VELOCITY = 0b00000000000000000000000000010000;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_ANGULAR_VELOCITY_BODY = 0b00000000000000000000000000100000;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_FORCE_BODY = 0b00000000000000000000000001000000;
    const uint16_t MAVLINK_SET_ATTITUDE_TARGET_TYPE_MASK_FORCE_NED = 0b00000000000000000000000010000000;
     
    //--------------------------------------------------------------------------
    // Waypoint Navigation Variables
    //--------------------------------------------------------------------------
    
    /// Current waypoint being navigated to (ENU coordinates)
    geometry_msgs::msg::PoseStamped current_waypoint_;
    
    /// Pending waypoint/path when mode change is in progress
    geometry_msgs::msg::PoseStamped pending_waypoint_; 
    bool has_pending_waypoint_{false};
    
    /// Waypoint navigation state
    bool waypoint_navigation_active_{false}; // Actively following waypoints
    bool waypoint_reached_{false};           // Current waypoint reached
    bool guided_mode_active_{false};         // Vehicle is in GUIDED mode
    
    /// Mode change tracking
    bool mode_change_requested_{false};      // Mode change has been requested
    rclcpp::Time mode_change_request_time_{}; // Time of last mode change request
    uint32_t mode_change_attempts_{0};       // Number of mode change attempts
    std::string requested_mode_{};           // The mode that was requested
    
    /// Waypoint acceptance radius (meters)
    double waypoint_acceptance_radius_{0.1};
    
    /// Position hold at last waypoint when queue is empty
    bool position_hold_active_{false};              // Vehicle is in POSHOLD mode
    geometry_msgs::msg::PoseStamped position_hold_waypoint_; // Position being held
    

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
     * @brief Set the update interval for MAVLink messages
     */
    void setMessageInterval(uint16_t message_id, float frequency_hz);
    
    /**
     * @brief Handle HEARTBEAT message
     * @param msg The received MAVLink message
     * @param sender_addr The sender's address
     */
    void handleHeartbeat(const mavlink_message_t& msg, const sockaddr_in& sender_addr);
    
    /**
     * @brief Handle LOCAL_POSITION_NED message
     * @param msg The received MAVLink message
     */
    void handleLocalPositionNed(const mavlink_message_t& msg);

    /**
     * @brief Handle GLOBAL_POSITION_INT message
     * @param msg The received MAVLink message
     */
    void handleGlobalPositionInt(const mavlink_message_t& msg);

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
     * @brief Process velocity commands from ROS
     */
    void velocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief Service callback for arming/disarming the vehicle.
     * @param request Service request containing boolean for arm (true) or disarm (false).
     * @param response Service response indicating success/failure.
     */
    void armingServiceCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     * @brief Service callback for setting the vehicle's flight mode.
     * @param request Service request containing the desired flight mode string.
     * @param response Service response indicating success/failure.
     */
    void flightModeServiceCallback(
        const std::shared_ptr<auv_core_helper::srv::SetFlightMode::Request> request,
        std::shared_ptr<auv_core_helper::srv::SetFlightMode::Response> response);

    /**
     * @brief Set the arm state
     * @param arm The desired arm state
     */
    void setArmState(bool arm_vehicle);

    /**
     * @brief Set the flight mode
     * @param mode The desired flight mode
     */
    void setFlightMode(const std::string& mode);

    /**
     * @brief Callback for receiving desired RC channels
     * @param msg The received RCChannels message
     */
    void rcChannelValuesDesiredCallback(const auv_core_helper::msg::RCChannels::SharedPtr msg);

    /**
     * @brief Set RC channel PWM values
     */
    void setRcChannelPwm(const uint16_t* rc_channel_values);
    
    /**
     * @brief Callback for receiving desired pose
     * @param msg The received PoseStamped message
     */
    void localPoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Callback for receiving desired velocity
     * @param msg The received Twist message
     */
    void localVelocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    

    /**
     * @brief Send waypoint to ArduSub in NED coordinates
     */
    void SetPositionTargetLocalNED(const mavlink_set_position_target_local_ned_t& position_target_);
    
    /**
     * @brief Send a waypoint in global coordinates
     * 
     * Sends waypoint using SET_POSITION_TARGET_GLOBAL_INT message to enable
     * global positioning and navigation.
     * 
     * @param lat_int Latitude (degrees * 1e7)
     * @param lon_int Longitude (degrees * 1e7)
     * @param alt Altitude in meters (negative for below sea level)
     */
    void sendGlobalWaypoint(int32_t lat_int, int32_t lon_int, float alt);
    
    /**
     * @brief Convert a global waypoint to MAVLink SET_POSITION_TARGET_GLOBAL_INT message
     * 
     * @param lat_int Latitude (degrees * 1e7)
     * @param lon_int Longitude (degrees * 1e7)
     * @param alt Altitude in meters
     * @param global_target Output structure for MAVLink message
     */
    void prepareGlobalPositionTarget(int32_t lat_int, int32_t lon_int, float alt, 
                                    mavlink_set_position_target_global_int_t& global_target);

    /**
     * @brief Send MAV_CMD_CONDITION_YAW command to set vehicle heading
     * 
     * This command sets the heading of the vehicle. It can be used in combination
     * with waypoints to control the facing direction.
     * 
     * @param heading_deg Target heading in degrees
     * @param is_relative If true, heading is relative to current heading
     * @param direction Direction to rotate: 1=clockwise, -1=counterclockwise, 0=shortest
     * @param angular_rate Angular rate for rotation (degrees/second)
     */
    void sendConditionYaw(float heading_deg, bool is_relative = false, int direction = 0, float angular_rate = 0.0f);                                

    /**
     * @brief Set the attitude target
     * @param attitude_target The desired attitude target
     */
    void SetAttitudeTarget(const mavlink_set_attitude_target_t& attitude_target_);

    /**
     * @brief Send MAV_CMD_DO_SET_HOME command to set the home position
     * 
     * This sets the home position of the vehicle, which is used as the reference
     * for RTL mode and relative positions.
     * 
     * @param latitude Latitude in degrees (use NAN to use current position)
     * @param longitude Longitude in degrees (use NAN to use current position)
     * @param altitude Altitude in meters (above MSL)
     * @param use_current If true, ignore lat/lon/alt and use current position
     */
    void sendSetHome(float latitude, float longitude, float altitude, bool use_current = false);

    /**
     * @brief Send MAV_CMD_OVERRIDE_GOTO command to interrupt current navigation
     * 
     * This commands the vehicle to immediately move to the specified position.
     * It can be used in emergency situations to redirect the vehicle.
     * 
     * @param position Target position in ENU coordinates
     * @param continue_cmd If true, the vehicle will continue executing mission after reaching position
     */
    void sendOverrideGoto(const geometry_msgs::msg::Point& position, bool continue_cmd = false);
};
