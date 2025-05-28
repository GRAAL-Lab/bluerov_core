#include "states/waypoint_navigation_state.hpp"


WayPointNavigationState::WayPointNavigationState(fsm::FSM* fsm) : BaseAUVState(fsm, "WAYPOINT_NAVIGATION") {}

fsm::retval WayPointNavigationState::OnEntry() noexcept{

    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Entering WayPoint Navigation State");
    ctrlData->armed_desired    = true;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::GUIDED;
    ctrlData->deisiredCtrlMode = auv_core_helper::BrigdeMode::PoseCtrl;

    waypoint_.latitude  = ctrlData->poseGoalGlobal(0);
    waypoint_.longitude = ctrlData->poseGoalGlobal(1);
    waypointDepth_      = ctrlData->poseGoalGlobal(2);
    isFacingGoal_       = false;
    return fsm::ok;
}

fsm::retval WayPointNavigationState::Execute() noexcept{
    current_.latitude  = ctrlData->poseActualGlobal(0);
    current_.longitude = ctrlData->poseActualGlobal(1);

    ctb::DistanceAndAzimuthRad(current_, waypoint_,distanceToGoal_, headingToGoal_);


    if (!isFacingGoal_){
        ctrlData->poseGoalGlobal(0) = ctrlData->poseActualGlobal(0);
        ctrlData->poseGoalGlobal(1) = ctrlData->poseActualGlobal(1);
        ctrlData->poseGoalGlobal(2) = ctrlData->poseActualGlobal(2);
        ctrlData->poseGoalGlobal(3) = 0;
        ctrlData->poseGoalGlobal(4) = 0;
        ctb::NormalizeAngle(headingToGoal_);
        ctrlData->poseGoalGlobal(5) = headingToGoal_;

        if (std::fabs(ctb::AngleDifference(ctrlData->poseActualGlobal(5), headingToGoal_)) < YAW_TOL){
            isFacingGoal_ = true;
        }
    }
    else{
        ctrlData->poseGoalGlobal(0) = waypoint_.latitude;
        ctrlData->poseGoalGlobal(1) = waypoint_.longitude;
        ctrlData->poseGoalGlobal(2) = waypointDepth_;
        ctrlData->poseGoalGlobal(3) = 0;
        ctrlData->poseGoalGlobal(4) = 0;
        ctb::NormalizeAngle(headingToGoal_);
        ctrlData->poseGoalGlobal(5) = headingToGoal_;
        if (distanceToGoal_ < DIST_TOL) {
            fsm_->SetNextState(States::HOLD);
            return fsm::ok;
        }
    }
    return fsm::ok;
}

fsm::retval WayPointNavigationState::OnExit() noexcept{

    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Exiting WAYPOINT_NAVIGATION");
    isFacingGoal_ = false;
    return fsm::ok;
}
