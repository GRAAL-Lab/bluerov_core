#include "states/hold_state.hpp"


//TO DO

// Constructor remains unchanged
HoldState::HoldState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "HOLD") {
}

// onEntry: Initialize the hold state
fsm::retval HoldState::OnEntry() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("HoldState"), "Control data is null!");
        return fsm::fail;
    }

    //arm vechicle
    ctrlData->armed_desired = true;
    ctrlData->flightMode_desired = "POSHOLD"; // Set flight mode to position hold
    //set vehicle to hold mode
    RCLCPP_INFO(rclcpp::get_logger("HoldState"), "Entering HOLD state");

  return fsm::ok;
}

// execute: Perform hold logic
fsm::retval HoldState::Execute() noexcept {
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("HoldState"), "Control data is null!");
        return fsm::fail;
    }
    return fsm::ok;
}

// onExit remains unchanged
fsm::retval HoldState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("HoldState"), "Exiting HOLD state");
    return fsm::ok;
}