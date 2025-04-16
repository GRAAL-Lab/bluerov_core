#include "states/waypoint_navigation_state.hpp"

//TO DO


// Constructor
WayPointNavigationState::WayPointNavigationState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "WAYPOINT_NAVIGATION") {}

// OnEntry: Initialize joystick state
fsm::retval WayPointNavigationState::OnEntry() noexcept {
    return fsm::ok;
}

// Execute: Process joystick input
fsm::retval WayPointNavigationState::Execute() noexcept {

    
}

// OnExit: Cleanup
fsm::retval WayPointNavigationState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Exiting WAYPOINT_NAVIGATION state");
    return fsm::ok;
}