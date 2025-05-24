#include "states/dive_state.hpp"

//TO DO

// Constructor
DiveState::DiveState(fsm::FSM* fsm): BaseAUVState(fsm, States::DIVE) {}

// OnEntry
fsm::retval DiveState::OnEntry() {
    RCLCPP_INFO(rclcpp::get_logger("DiveState"), "Entering Dive State");
    ctrlData->armed_desired = true;
    ctrlData->deisiredCtrlMode = auv_core_helper::BrigdeMode::PoseCtrl;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::GUIDED;
    ctrlData->poseGoalGlobal(0) =  ctrlData->poseActualGlobal(0);
    ctrlData->poseGoalGlobal(1) =  ctrlData->poseActualGlobal(1);
    ctrlData->poseGoalGlobal(2) = ctrlData->poseActualGlobal(2)-1;
    ctrlData->poseGoalGlobal(3) = 0;
    ctrlData->poseGoalGlobal(4) = 0;
    ctrlData->poseGoalGlobal(5) = ctrlData->poseActualGlobal(5);
    return fsm::ok;
}

// Execute
fsm::retval DiveState::Execute() {
    return fsm::ok;
}

// OnExit
fsm::retval DiveState::OnExit() {
    return fsm::ok;
}
