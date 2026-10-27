#pragma once

#include "states/base_auv_state.hpp"


class LockDvlState : public BaseAUVState {
public:

    explicit LockDvlState(fsm::FSM* fsm);


    fsm::retval OnEntry() noexcept override;

    fsm::retval Execute() noexcept override;

    fsm::retval OnExit() noexcept override;
};
