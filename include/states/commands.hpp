#ifndef AUV_STATES_COMMANDS_HPP
#define AUV_STATES_COMMANDS_HPP

#include <string>

/// Namespace containing state identifiers for the AUV FSM.
namespace States {
    constexpr char IDLE[] = "IDLE"; ///< AUV is in an idle state.
    constexpr char HOLD[] = "HOLD"; ///< AUV is holding its current position.
    constexpr char WAYPOINT_NAVIGATION[] = "WAYPOINT_NAVIGATION"; ///< AUV is navigating to a waypoint.
    constexpr char SURFACE[] = "SURFACE"; ///< AUV is surfacing.
    constexpr char DIVE[] = "DIVE"; ///< AUV is diving.
    constexpr char PATH_FOLLOWING[] = "PATH_FOLLOWING"; ///< AUV is following a planned path.
}

#endif // AUV_STATES_COMMANDS_HPP
