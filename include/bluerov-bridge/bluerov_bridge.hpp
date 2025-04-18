#pragma once

#include <rclcpp/rclcpp.hpp>

// ROS 2 message headers
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"         // For waypoint poses
#include "nav_msgs/msg/path.hpp"                     // For path of waypoints
#include "std_msgs/msg/bool.hpp"                     // For waypoint reached notification

// AUV-specific topic names
#include "auv_msgs_ros2/topicnames.hpp"

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
#include <queue>                                     // For waypoint queue

/**
 * @brief A ROS 2 node that interfaces with ArduPilot/BlueROV using MAVLink protocol.
 *
 * This bridge enables:
 * 1. Manual control by passing velocity commands to ArduSub
 * 2. Waypoint navigation in GUIDED mode (single waypoint or path)
 * 3. Position holding at the last waypoint
 * 
 * Features:
 * - Coordinate transformations between NED (ArduSub) and ENU (ROS)
 * - Automatic mode switching based on flight state
 * - Queue-based waypoint following
 * - Position hold after waypoint completion
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
    // ROS Publishers & Subscribers
    //--------------------------------------------------------------------------
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr poseActualPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityActualPublisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypointReachedPublisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocityDesiredSubscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kclStateSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypointSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSubscription_;

    //--------------------------------------------------------------------------
    // Timers
    //--------------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr data_timer_;         // Timer for MAVLink data reception
    rclcpp::TimerBase::SharedPtr control_loop_timer_; // Timer for manual control
    rclcpp::TimerBase::SharedPtr waypoint_timer_;     // Timer for waypoint navigation

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
    bool is_armed_{false};              // Flag indicating if vehicle is armed

    //--------------------------------------------------------------------------
    // State Variables
    //--------------------------------------------------------------------------

    /// [x, y, z, roll, pitch, yaw] in NED
    Eigen::Matrix<double, 6, 1> poseActual_{Eigen::Matrix<double, 6, 1>::Zero()};

    /// [vx, vy, vz, rollspeed, pitchspeed, yawspeed]
    Eigen::Matrix<double, 6, 1> velocityActual_{Eigen::Matrix<double, 6, 1>::Zero()};

    /// Home pose in NED
    Eigen::Matrix<double, 6, 1> poseHome_{Eigen::Matrix<double, 6, 1>::Zero()};
    bool home_pose_set_{false};

    /// Desired velocity [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
    Eigen::Matrix<double, 6, 1> velocityDesired_{Eigen::Matrix<double, 6, 1>::Zero()};   // [m/s, m/s, m/s, rad/s, rad/s, rad/s]
    Eigen::Matrix<double, 6, 1> velocityDesiredPwm_{Eigen::Matrix<double, 6, 1>::Zero()}; // [PWM, PWM, PWM, PWM, PWM, PWM]

    /// Velocity limits
    Eigen::Matrix<double, 6, 1> maxVelocities_{2.0, 1.8, 0.55, 2.0, 2.2, 1.85}; // [m/s, m/s, m/s, rad/s, rad/s, rad/s]
    Eigen::Matrix<double, 6, 1> minVelocities_{-2.0, -1.8, -0.55, -2.0, -2.2, -1.85}; // [m/s, m/s, m/s, rad/s, rad/s, rad/s]

    /// KCL state
    std::string kcl_state_{};

    /// Depth filtering
    bool depth_initialized_{false};
    double depth_filtered_;
    double alpha_depth_;
    
    //--------------------------------------------------------------------------
    // Waypoint Navigation Variables
    //--------------------------------------------------------------------------
    
    /// Queue of waypoints (stored in ENU coordinates)
    std::queue<geometry_msgs::msg::PoseStamped> waypoint_queue_;
    
    /// Current waypoint being navigated to (ENU coordinates)
    geometry_msgs::msg::PoseStamped current_waypoint_;
    
    /// Pending waypoint/path when mode change is in progress
    geometry_msgs::msg::PoseStamped pending_waypoint_; 
    nav_msgs::msg::Path pending_path_;
    bool has_pending_waypoint_{false};
    bool has_pending_path_{false};
    
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
     * @brief Request data streams from ArduSub
     */
    void requestDataStreams();
    
    /**
     * @brief Receive and process MAVLink data
     */
    void receiveData();
    
    /**
     * @brief Main control loop for manual control mode
     */
    void controlLoop();

    /**
     * @brief Process velocity commands from ROS
     */
    void velocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief Process state change requests
     */
    void kclStateCallback(const std_msgs::msg::String::SharedPtr msg);
    
    /**
     * @brief Process single waypoint requests
     */
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief Process path (multiple waypoints) requests
     */
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    
    /**
     * @brief Timer callback for waypoint navigation progress
     */
    void waypointNavigationTimer();
    
    /**
     * @brief Process the next waypoint in the queue
     */
    void processNextWaypoint();
    
    /**
     * @brief Check if current waypoint has been reached
     */
    bool isWaypointReached();
    
    /**
     * @brief Check if the vehicle is armed
     * @return true if armed, false otherwise
     */
    bool isArmed();
    
    /**
     * @brief Send waypoint to ArduSub in NED coordinates
     */
    void sendWaypointToArdupilot(const geometry_msgs::msg::PoseStamped& waypoint);
    
    /**
     * @brief Convert from ENU to NED coordinate system
     */
    void convertENUtoNED(const geometry_msgs::msg::PoseStamped& enu, mavlink_set_position_target_local_ned_t& ned);

    /**
     * @brief Set flight mode without changing arm state
     * 
     * This method sets the vehicle flight mode without changing the arming state.
     * It supports these modes: "MANUAL", "STABILIZE", "ALT_HOLD", "GUIDED", "POSHOLD"
     * 
     * @param mode The desired flight mode
     */
    void setFlightMode(const std::string& mode);
    
    /**
     * @brief Arm the vehicle in specified mode
     * 
     * Sends command to arm the vehicle. If mode is not empty and not "CURRENT",
     * it will also set the vehicle to the specified mode after arming.
     * 
     * @param mode The desired flight mode after arming (empty or "CURRENT" to keep current mode)
     */
    void arm(const std::string& mode = "MANUAL");
    
    /**
     * @brief Disarm the vehicle
     */
    void disarm();

    /**
     * @brief Convert desired velocities to PWM values
     */
    Eigen::VectorXd velocityToPwm(const Eigen::VectorXd& velocities, const Eigen::VectorXd& maxVelocities, const Eigen::VectorXd& minVelocities);
    
    /**
     * @brief Set RC channel PWM values
     */
    void setRcChannelPwm(const Eigen::VectorXd& velocityDesiredPwm);
    
    /**
     * @brief Send a MAVLink message to ArduSub
     */
    void sendMavlinkMessage(const mavlink_message_t& msg);
    
    /**
     * @brief Set the update interval for MAVLink messages
     */
    void setMessageInterval(uint16_t message_id, float frequency_hz);
    
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
};

