#include "states/waypoint_navigation_state.hpp"

//TO DO


// Constructor
WayPointNavigationState::WayPointNavigationState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "WAYPOINT_NAVIGATION") {}

// OnEntry: Initialize joystick state
fsm::retval WayPointNavigationState::OnEntry() noexcept {
    //ARM
    //SETGUIDED MODE
    //set waipoint desired
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Entering WAYPOINT_NAVIGATION state");
    return fsm::ok;
}

// Execute: Process joystick input
fsm::retval WayPointNavigationState::Execute() noexcept {
    //calulate heading diesred
    //set heading desired
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Executing WAYPOINT_NAVIGATION state");

    
}

// OnExit: Cleanup
fsm::retval WayPointNavigationState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Exiting WAYPOINT_NAVIGATION state");
    //put vechile in hold mode/ position hold set fsm in hold state
    return fsm::ok;
}