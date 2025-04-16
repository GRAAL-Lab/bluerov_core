#pragma once

#include "states/base_auv_state.hpp"

/// The `IdleState` class represents an idle state in which the AUV performs no active operations.
/// All control outputs (e.g., velocities, goals) are reset to zero.
class IdleState : public BaseAUVState {
public:
    /// Constructor for the `IdleState` class.
    /// @param fsm Pointer to the FSM controlling this state.
    explicit IdleState(fsm::FSM* fsm);

    /// Called when the AUV enters the idle state.
    /// Resets control outputs and prepares the AUV for an idle condition.
    fsm::retval OnEntry() noexcept override;

    /// Called repeatedly while the AUV is in the idle state.
    /// Performs no active operations.
    fsm::retval Execute() noexcept override;

    /// Called when the AUV exits the idle state.
    /// Can be used to prepare for transitioning to another state.
    fsm::retval OnExit() noexcept override;
};
