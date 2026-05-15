#include "states/lock_dvl_state.hpp"
#include <iostream>


// Constructor
LockDvlState::LockDvlState(fsm::FSM* fsm): BaseAUVState(fsm, "LOCK_DVL") {}

// onEntry: Reset all control data
fsm::retval LockDvlState::OnEntry() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("LockDvlState"), "Control data is null!");
        return fsm::fail;
    }
    // //disarm vehicle
    // ctrlData->armed_desired = false;
    // ctrlData->flightMode_desired = auv_core_helper::FlightMode::POSHOLD;
    // ctrlData->deisiredCtrlMode = auv_core_helper::BrigdeMode::PoseCtrl;
    return fsm::ok;
}

// execute: Perform no active operations
fsm::retval LockDvlState::Execute() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("LockDvlState"), "Control data is null!");
        return fsm::fail;
    }
    return fsm::ok;
}

// onExit: Log the state exit
fsm::retval LockDvlState::OnExit() noexcept {
    return fsm::ok;
}
