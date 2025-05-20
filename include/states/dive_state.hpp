#pragma once

#include "states/base_auv_state.hpp"

class DiveState : public BaseAUVState {
private:

public:
    explicit DiveState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
};