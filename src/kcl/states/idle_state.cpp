#include "states/idle_state.hpp"
#include <iostream>


//TO DO

// Constructor
IdleState::IdleState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "IDLE") {}

// onEntry: Reset all control data
fsm::retval IdleState::OnEntry() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("IdleState"), "Control data is null!");
        return fsm::fail;
    }
    //disarm vehicle
    ctrlData->armed_desired = false;
    return fsm::ok;
}

// execute: Perform no active operations
fsm::retval IdleState::Execute() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("IdleState"), "Control data is null!");
        return fsm::fail;
    }
    // The idle state does not perform any operations
    return fsm::ok;
}

// onExit: Log the state exit
fsm::retval IdleState::OnExit() noexcept {
    return fsm::ok;
}
