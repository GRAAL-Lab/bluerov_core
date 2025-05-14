#pragma once

// Standard library headers
#include <memory>
#include <string>

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>

// AUV-specific headers
#include "kcl/data_structs.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/helper_lib.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// State headers
#include "states/base_auv_state.hpp"
#include "states/idle_state.hpp"
#include "states/hold_state.hpp"
#include "states/waypoint_navigation_state.hpp"
#include "states/surface_state.hpp"
#include "states/path_following_state.hpp"
#include "states/commands.hpp"

// AUV-specific topic names
#include "auv_core_helper/topicnames.hpp"

// AUV-specific message types between mission control and the AUV
#include "auv_core_helper/action/set_kcl.hpp"

// Graal library 
#include "fsm/fsm.h"
#include "rml/Functions.h"

class KCL : public rclcpp::Node {
public:
    explicit KCL();

    /// Executes the FSM by running the state transitions and actions.
    void ExecuteFSM();

private:
    // --------------------
    // Finite State Machine
    // --------------------
    fsm::FSM fsm_; ///< The finite state machine instance.

    // --------------------
    // State Variables
    // --------------------
    std::string desiredState_;
    
    // State objects
    std::unique_ptr<IdleState> idleState_;
    std::unique_ptr<HoldState> holdState_;
    std::unique_ptr<WayPointNavigationState> wayPointNavigationState_;
    std::unique_ptr<SurfaceState> surfaceState_;
    std::unique_ptr<PathFollowingState> pathFollowingState_;

    // --------------------
    // ROS 2 Publishers
    // --------------------
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr poseGoalPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityDesiredPublisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePublisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;

    // --------------------
    // ROS 2 Subscriptions
    // --------------------
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseActualSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocityActualSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr accelerationActualSubscription_;

    // --------------------
    // ROS 2 Services
    // --------------------
    // rclcpp::Service<auv_core_helper::srv::ControlCommand>::SharedPtr controlCommandService_;

    // --------------------
    // ROS 2 Action Server
    // --------------------
    rclcpp_action::Server<auv_core_helper::action::SetKCL>::SharedPtr KCLSetter_;
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const auv_core_helper::action::SetKCL::Goal> goal);
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<auv_core_helper::action::SetKCL>> goal_handle);




    // --------------------
    // Timer
    // --------------------
    rclcpp::TimerBase::SharedPtr fsmTimer_;

    // --------------------
    // Shared Data
    // --------------------
    std::shared_ptr<auv::ControlData> ctrlData_; ///< Shared pointer to the control data struct.

    // --------------------
    // Private Functions
    // --------------------
    /// Set up FSM transitions and state machine logic.
    void SetupTransitions();

    /// Callback for actual pose data.
    void PoseActualCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    
    /// Callback for actual velocity data.
    void VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /// Callback for actual acceleration data.
    void AccelerationActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /// Callback for control command service.
    void HandleSetKCL(const std::shared_ptr<rclcpp_action::ServerGoalHandle<auv_core_helper::action::SetKCL>> goal_handle);

    

};
