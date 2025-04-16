#pragma once

#include "states/base_auv_state.hpp"

class SurfaceState : public BaseAUVState {
private:

public:
    explicit SurfaceState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
};