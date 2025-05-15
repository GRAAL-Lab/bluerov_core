#include "states/waypoint_navigation_state.hpp"

//TO DO


// Constructor
WayPointNavigationState::WayPointNavigationState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "WAYPOINT_NAVIGATION") {}

// OnEntry: Initialize joystick state
fsm::retval WayPointNavigationState::OnEntry() noexcept {
    ctrlData->armed_desired = true;
    ctrlData->flightMode_desired = "GUIDED";

    //set waipoint desired
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Entering WAYPOINT_NAVIGATION state");
    return fsm::ok;
}

// Execute: Process joystick input
fsm::retval WayPointNavigationState::Execute() noexcept {

    current.latitude = ctrlData->poseActualGlobal(0);
    current.longitude = ctrlData->poseActualGlobal(1);
    goal.latitude = ctrlData->poseGoalGlobal(0);
    goal.longitude = ctrlData->poseGoalGlobal(1);
    ctb::DistanceAndAzimuthRad(current, goal, distanceToGoal, headingToGoal);

    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Distance to goal: %f", distanceToGoal);
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Heading to goal: %f", headingToGoal);


    if (isFacingGoal_) {
        //set the desired heading by publish on pose desired global only in z and yaw
        ctrlData->poseGoalGlobal(0) = ctrlData->poseActualGlobal(0);
        ctrlData->poseGoalGlobal(1) = ctrlData->poseActualGlobal(1);
        ctrlData->poseGoalGlobal(2) = 0;
        ctrlData->poseGoalGlobal(3) = 0;
        ctrlData->poseGoalGlobal(4) = 0;
        ctrlData->poseGoalGlobal(5) = -headingToGoal;
        if (-headingToGoal == ctrlData->poseActualGlobal(5)) {
            isFacingGoal_ = true;
        }
    } else {
        //start going to goal
        ctrlData->poseGoalGlobal(0) = goal.latitude;
        ctrlData->poseGoalGlobal(1) = goal.longitude;
        ctrlData->poseGoalGlobal(2) = 0;
        ctrlData->poseGoalGlobal(3) = 0;
        ctrlData->poseGoalGlobal(4) = 0;
        ctrlData->poseGoalGlobal(5) = headingToGoal;
        if (distanceToGoal < 0.1) {
            fsm_->SetNextState(States::HOLD);
            return fsm::ok;
        }
    }
    return fsm::ok;
}

// OnExit: Cleanup
fsm::retval WayPointNavigationState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("WayPointNavigationState"), "Exiting WAYPOINT_NAVIGATION state");
    //put vechile in hold mode/ position hold set fsm in hold state
    return fsm::ok;
}