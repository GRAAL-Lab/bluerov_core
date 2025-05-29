#pragma once

#include "states/base_auv_state.hpp"
#include <vector>
#include <string>
#include <memory>

class WayPointNavigationState : public BaseAUVState
{
private:
    bool isFacingGoal_{false};          ///< true once yaw is aligned
    ctb::LatLong waypoint_;             ///< immutable mission waypoint
    double waypointDepth_{0.0};         ///< depth of the waypoint
    ctb::LatLong current_;              ///< live vehicle position

    double distanceToGoal_{0.0};
    double headingToGoal_{0.0};

    double targetHeading_{0.0};         ///< freeze heading calculated on entry

    constexpr static double YAW_TOL   = 0.05;  ///< 3 deg
    constexpr static double DIST_TOL  = 0.15;  ///< 10 cm

public:
    explicit WayPointNavigationState(fsm::FSM* fsm);

    fsm::retval OnEntry()   noexcept override;
    fsm::retval Execute()   noexcept override;
    fsm::retval OnExit()    noexcept override;
};
