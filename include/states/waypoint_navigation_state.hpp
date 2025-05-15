#pragma once

#include "states/base_auv_state.hpp"
#include <vector>
#include <string>
#include <memory>


class WayPointNavigationState : public BaseAUVState {
private:
bool isFacingGoal_{false}; ///< Flag to indicate if the vehicle is heading towards the goal.
ctb::LatLong current; ///< Current position of the vehicle in latitude and longitude.
ctb::LatLong goal; ///< Goal position of the vehicle in latitude and longitude.
double distanceToGoal; ///< Distance to the goal position.
double headingToGoal; ///< Heading to the goal position in radians.
double distanceToGoalThreshold = 0.1; ///< Threshold distance to consider the goal reached.



public:
    explicit WayPointNavigationState(fsm::FSM* fsm);

    fsm::retval OnEntry() noexcept override;
    fsm::retval Execute() noexcept override;
    fsm::retval OnExit() noexcept override;

private:
};
