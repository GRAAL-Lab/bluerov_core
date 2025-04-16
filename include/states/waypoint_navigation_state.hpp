#pragma once

#include "states/base_auv_state.hpp"
#include <vector>
#include <string>
#include <memory>


class WayPointNavigationState : public BaseAUVState {
private:




public:
    explicit WayPointNavigationState(fsm::FSM* fsm);

    fsm::retval OnEntry() noexcept override;
    fsm::retval Execute() noexcept override;
    fsm::retval OnExit() noexcept override;

private:
};
