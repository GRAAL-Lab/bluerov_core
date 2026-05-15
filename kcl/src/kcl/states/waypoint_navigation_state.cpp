#include "states/waypoint_navigation_state.hpp"



/* ──────────────────────────────────────────────────────────────── */

WayPointNavigationState::WayPointNavigationState(fsm::FSM* fsm)
: BaseAUVState(fsm, "WAYPOINT_NAVIGATION")
{}

/* helper: Euclidean distance in NED given horiz & vert components */
static inline double hypot3(double horizontal, double vertical)
{
    return std::sqrt(horizontal*horizontal + vertical*vertical);
}


fsm::retval WayPointNavigationState::OnEntry() noexcept
{
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Entering WayPoint Navigation State");

    /* arm vehicle & select flight mode */
    ctrlData->armed_desired      = true;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::GUIDED;
    ctrlData->deisiredCtrlMode   = auv_core_helper::BrigdeMode::PoseCtrl;

    /* ── cache waypoint (lat/lon, depth) ────────────────────────── */
    waypoint_.latitude  = ctrlData->poseGoalGlobal(0);
    waypoint_.longitude = ctrlData->poseGoalGlobal(1);
    waypointDepth_      = ctrlData->poseGoalGlobal(2);

    /* current position                                              */
    current_.latitude  = ctrlData->poseActualGlobal(0);
    current_.longitude = ctrlData->poseActualGlobal(1);

    /* horiz. distance + heading                                     */
    ctb::DistanceAndAzimuthRad(current_, waypoint_,
                               distanceToGoal_,      /* horiz [m] */
                               targetHeading_);
    ctb::NormalizeAngle(targetHeading_);

    /* store full initial 3-D distance                               */
    const double vert0 = std::fabs(ctrlData->poseActualGlobal(2)
                                   - std::fabs(waypointDepth_));
    initialDist3D_     = hypot3(distanceToGoal_, vert0);
    initialDist3D_     = std::max(initialDist3D_, 0.01);   // avoid /0

    /* progress starts at 0 %                                        */
    ctrlData->actionProgress = 0.0;
    ctrlData->actionSuccess  = false;
    ctrlData->actionFailed   = false;

    /* If already facing goal skip the turn phase                    */
    isFacingGoal_ = (distanceToGoal_ < DIST_TOL);

    return fsm::ok;
}


fsm::retval WayPointNavigationState::Execute() noexcept
{
    // RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Executing WayPoint Navigation State");

    /* ── 1.  update current position & horiz distance ───────────── */
    current_.latitude  = ctrlData->poseActualGlobal(0);
    current_.longitude = ctrlData->poseActualGlobal(1);

    double unusedAzimuth;
    ctb::DistanceAndAzimuthRad(current_, waypoint_,
                               distanceToGoal_,
                               unusedAzimuth);

    const double depthErr =
        std::fabs(ctrlData->poseActualGlobal(2) - std::fabs(waypointDepth_));

    /* ── 1.b  update progress (0-100 %) ─────────────────────────── */
    const double remaining3D = hypot3(distanceToGoal_, depthErr);
    const double progressPct =
        std::clamp(100.0 * (1.0 - remaining3D / initialDist3D_), 0.0, 100.0);
    ctrlData->actionProgress = progressPct;

    /* ── 2.  Phase A: yaw-align if not yet facing goal ──────────── */
    if (!isFacingGoal_)
    {
        ctrlData->poseGoalGlobal << current_.latitude,
                                   current_.longitude,
                                   ctrlData->poseActualGlobal(2),
                                   0, 0,
                                   targetHeading_;
        ctb::NormalizeAngle(targetHeading_);   

        if (std::fabs(ctb::AngleDifference(ctrlData->poseActualGlobal(5),
                                           targetHeading_)) < YAW_TOL)
            isFacingGoal_ = true;

        return fsm::ok;          // stay in yaw-align phase
    }

    /* ── 3.  Phase B: translate / dive ──────────────────────────── */
    if (distanceToGoal_ > DIST_TOL)            /* need XY motion */
    {
        ctrlData->poseGoalGlobal << waypoint_.latitude,
                                   waypoint_.longitude,
                                   waypointDepth_,
                                   0, 0,
                                   targetHeading_;
    }
    else                                        /* XY reached     */
    {
        ctrlData->poseGoalGlobal << current_.latitude,
                                   current_.longitude,
                                   waypointDepth_,
                                   0, 0,
                                   ctrlData->poseActualGlobal(5);
    }

    /* ── 4.  Transition & success flag ──────────────────────────── */
    if (distanceToGoal_ < DIST_TOL && depthErr < DIST_TOL)
    {
        ctrlData->actionSuccess = true;         // ❸ inform KCL
        fsm_->SetNextState(States::HOLD);
    }


    return fsm::ok;
}


fsm::retval WayPointNavigationState::OnExit() noexcept
{
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Exiting WAYPOINT_NAVIGATION");

    isFacingGoal_ = false;
    return fsm::ok;
}
