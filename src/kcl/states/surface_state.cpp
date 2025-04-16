#include "states/surface_state.hpp"

//TO DO

// Constructor
SurfaceState::SurfaceState(fsm::FSM* fsm)
    : BaseAUVState(fsm, States::SURFACE) {}

// OnEntry
fsm::retval SurfaceState::OnEntry() {
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
