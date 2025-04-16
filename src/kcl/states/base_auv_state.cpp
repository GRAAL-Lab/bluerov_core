#include "states/base_auv_state.hpp"
#include "kcl/data_structs.hpp"

// Constructor
BaseAUVState::BaseAUVState(fsm::FSM* fsm, const std::string& name) 
    : fsm_(fsm), stateName_(name) {
    // Initialization or logging if needed
}
