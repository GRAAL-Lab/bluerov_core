#include "states/waypoint_navigation_state.hpp"



WayPointNavigationState::WayPointNavigationState(fsm::FSM* fsm): BaseAUVState(fsm, "WAYPOINT_NAVIGATION") {}


fsm::retval WayPointNavigationState::OnEntry() noexcept
{
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"),
                "Entering WayPoint Navigation State");

    /* arm vehicle & select flight mode */
    ctrlData->armed_desired     = true;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::GUIDED;
    ctrlData->deisiredCtrlMode   = auv_core_helper::BrigdeMode::PoseCtrl;

    /* cache the waypoint once */
    waypoint_.latitude  = ctrlData->poseGoalGlobal(0);
    waypoint_.longitude = ctrlData->poseGoalGlobal(1);
    waypointDepth_      = ctrlData->poseGoalGlobal(2);

    /* compute a single travel-heading that will not change later          */
    current_.latitude  = ctrlData->poseActualGlobal(0);
    current_.longitude = ctrlData->poseActualGlobal(1);
    ctb::DistanceAndAzimuthRad(current_, waypoint_, distanceToGoal_, targetHeading_);
    ctb::NormalizeAngle(targetHeading_);

    /* If we are already at the horizontal goal we can skip the “turn” phase */
    isFacingGoal_ = (distanceToGoal_ < DIST_TOL);

    return fsm::ok;
}


fsm::retval WayPointNavigationState::Execute() noexcept
{
    RCLCPP_DEBUG(rclcpp::get_logger("WayPointNavigationState"),
                 "Executing WayPoint Navigation State");

    /* 1 ─ Update current position and horizontal distance (heading unused) */
    
    current_.latitude  = ctrlData->poseActualGlobal(0);
    current_.longitude = ctrlData->poseActualGlobal(1);

    double unusedAzimuth;
    ctb::DistanceAndAzimuthRad(current_, waypoint_,
                            distanceToGoal_,      // ← still stored
                            unusedAzimuth);       // ← dummy l-value
    

        // print poseActualGlobal(2) and waypointDepth_
    const double depthErr = std::fabs(ctrlData->poseActualGlobal(2) - std::fabs(waypointDepth_));


    /* 2 ─ Phase A : turn in place toward the waypoint (only if required)  */
    //print is facing goal
    std::cout << "Is facing goal: " << isFacingGoal_ << std::endl;
    if (!isFacingGoal_)
    {
        /* hold current X-Y-Z, command yaw only                               */
        ctrlData->poseGoalGlobal << current_.latitude,
                                   current_.longitude,
                                   ctrlData->poseActualGlobal(2),
                                   0, 0,
                                   targetHeading_;

        if (std::fabs(ctb::AngleDifference(ctrlData->poseActualGlobal(5),
                                           targetHeading_)) < YAW_TOL)
            isFacingGoal_ = true;

        return fsm::ok;   // stay in this phase until yaw is aligned
    }

    /* 3 ─ Phase B : translate &/or dive                                     */
    if (distanceToGoal_ > DIST_TOL)          /* still need horizontal motion? */
    {
        /* “normal” leg: drive toward waypoint and dive en-route */
        ctrlData->poseGoalGlobal << waypoint_.latitude,
                                   waypoint_.longitude,
                                   waypointDepth_,
                                   0, 0,
                                   targetHeading_;             // keep constant yaw
    }
    else                                      /* horizontal goal reached       */
    {
        /* HOLD X-Y, finish the dive, keep whatever yaw we currently have.   */
        ctrlData->poseGoalGlobal << current_.latitude,
                                   current_.longitude,
                                   waypointDepth_,
                                   0, 0,
                                   ctrlData->poseActualGlobal(5);
    }
    /* 4 ─ Transition check */
    if (distanceToGoal_ < DIST_TOL && depthErr < DIST_TOL)
        fsm_->SetNextState(States::HOLD);

    return fsm::ok;
}


fsm::retval WayPointNavigationState::OnExit() noexcept
{
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"),
                "Exiting WAYPOINT_NAVIGATION");
    isFacingGoal_ = false;        // reset local state
    return fsm::ok;
}
