#include "states/surface_state.hpp"


SurfaceState::SurfaceState(fsm::FSM* fsm): BaseAUVState(fsm, States::SURFACE) {}

// OnEntry
fsm::retval SurfaceState::OnEntry() {
    ctrlData->armed_desired = true;
    ctrlData->flightMode_desired = "SURFACE"; // Set flight mode to surface
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
