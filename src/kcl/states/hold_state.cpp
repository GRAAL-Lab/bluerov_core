#include "states/hold_state.hpp"


HoldState::HoldState(fsm::FSM* fsm) : BaseAUVState(fsm, "HOLD") {
}

fsm::retval HoldState::OnEntry() noexcept {

    RCLCPP_INFO(rclcpp::get_logger("HoldState"), "Entering Hold State");
    ctrlData->armed_desired = true;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::POSHOLD;
    ctrlData->deisiredCtrlMode = auv_core_helper::BrigdeMode::PoseCtrl;
    
  return fsm::ok;
}

// execute: Perform hold logic
fsm::retval HoldState::Execute() noexcept {
    return fsm::ok;
}

// onExit remains unchanged
fsm::retval HoldState::OnExit() noexcept {
    return fsm::ok;
}