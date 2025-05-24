#include "states/surface_state.hpp"


SurfaceState::SurfaceState(fsm::FSM* fsm): BaseAUVState(fsm, States::SURFACE) {}

// OnEntry
fsm::retval SurfaceState::OnEntry() {
    ctrlData->armed_desired = true;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::SURFACE;
    ctrlData->deisiredCtrlMode = auv_core_helper::BrigdeMode::PoseCtrl;
    return fsm::ok;
}

// Execute
fsm::retval SurfaceState::Execute() {
    return fsm::ok;
}

// OnExit
fsm::retval SurfaceState::OnExit() {
    return fsm::ok;
}
