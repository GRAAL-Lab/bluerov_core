#pragma once

#include "states/base_auv_state.hpp"
#include <iostream>


class HoldState : public BaseAUVState {
private:

public:
    /// Constructor for the `HoldState` class.
    /// @param fsm Pointer to the FSM controlling this state.
    explicit HoldState(fsm::FSM* fsm);

    /// Called when the AUV enters the hold state.
    /// @return Return value indicating the result of entering the state.
    fsm::retval OnEntry() noexcept override;

    /// Called repeatedly while the AUV is in the hold state.
    /// @return Return value indicating the result of execution.
    fsm::retval Execute() noexcept override;

    /// Called when the AUV exits the hold state.
    /// @return Return value indicating the result of exiting the state.
    fsm::retval OnExit() noexcept override;
};
