#include "states/trajectory_following_state.hpp"

#include <cmath>
#include <iostream>

TrajectoryFollowingState::TrajectoryFollowingState(fsm::FSM* fsm)
    : BaseAUVState(fsm, States::TRAJECTORY_FOLLOWING),
      t_total_(0.0),
      t_current_start_(0.0) {}

fsm::retval TrajectoryFollowingState::OnEntry() {
    RCLCPP_INFO(rclcpp::get_logger("TrajectoryFollowingState"), "Entering TRAJECTORY_FOLLOWING state");

    ctrlData->armed_desired = true;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::GUIDED;
    ctrlData->deisiredCtrlMode = auv_core_helper::BrigdeMode::VelCtrl;

    t_total_ = ctrlData->tpGoalTime;
    t_current_start_ = ctrlData->timeActual.seconds();
    pose_initial_ = ctrlData->poseActualLocal;

    pose_goal_ = ctrlData->poseGoalLocal;
    return fsm::ok;
}

fsm::retval TrajectoryFollowingState::Execute() {
    double t_current = ctrlData->timeActual.seconds() - t_current_start_;

    if (t_total_ > 0.0) {
        ctrlData->actionProgress = std::min(100.0, (t_current / t_total_) * 100.0);
    }

    if (t_total_ <= 0.0) {
        ctrlData->velocityDesiredNED.setZero();
        ctrlData->actionProgress = 0.0;
        ctrlData->actionFailed = true;
        ctrlData->actionMessage = "Trajectory time must be > 0";
        fsm_->SetNextState(States::HOLD);
        return fsm::ok;
    }

    if (t_current >= t_total_) {
        ctrlData->velocityDesiredNED.setZero();
        ctrlData->actionProgress = 100.0;

        if ((ctrlData->poseActualLocal - pose_goal_).norm() < 0.5) {
            std::cout << "Time elapsed, goal reached." << std::endl;
            ctrlData->actionSuccess = true;
            ctrlData->actionMessage = "Trajectory goal reached.";
        } else {
            std::cout << "Time elapsed, goal not reached." << std::endl;
            ctrlData->actionFailed = true;
            ctrlData->actionMessage = "Trajectory time elapsed before reaching goal.";
        }
        fsm_->SetNextState(States::HOLD);
        return fsm::ok;
    }

    Eigen::Matrix<double, 6, 1> vel_world_euler_rates =
        FindNextTrajectoryPoint(pose_initial_, pose_goal_, t_total_, t_current);

    Eigen::Vector3d v_world = vel_world_euler_rates.head<3>();
    Eigen::Vector3d euler_rates = vel_world_euler_rates.tail<3>();

    double roll = ctrlData->poseActualLocal(3);
    double pitch = ctrlData->poseActualLocal(4);
    double yaw = ctrlData->poseActualLocal(5);

    rml::EulerRPY rpy(roll, pitch, yaw);

    Eigen::Matrix3d R = rpy.ToRotationMatrix().matrix();
    Eigen::Vector3d v_body = R.transpose() * v_world;

    Eigen::Vector3d w_body = rpy.Omega(euler_rates);

    ctrlData->velocityDesiredNED(0) = v_body.x();
    ctrlData->velocityDesiredNED(1) = v_body.y();
    ctrlData->velocityDesiredNED(2) = v_body.z();
    ctrlData->velocityDesiredNED(3) = w_body.x();
    ctrlData->velocityDesiredNED(4) = w_body.y();
    ctrlData->velocityDesiredNED(5) = w_body.z();

    return fsm::ok;
}

fsm::retval TrajectoryFollowingState::OnExit() {
    ctrlData->velocityDesiredNED.setZero();
    return fsm::ok;
}

Eigen::Matrix<double, 6, 1> TrajectoryFollowingState::FindNextTrajectoryPoint(
    const Eigen::Matrix<double, 6, 1>& pose_initial,
    const Eigen::Matrix<double, 6, 1>& pose_goal,
    double t_total,
    double t_current) const {

    Eigen::Matrix<double, 6, 1> velocity_desired;
    double t_ratio = t_current / t_total;

    for (int i = 0; i < 6; ++i) {
        double delta_pose = pose_goal[i] - pose_initial[i];
        velocity_desired[i] =
            (30 * std::pow(t_ratio, 2) - 60 * std::pow(t_ratio, 3) + 30 * std::pow(t_ratio, 4))
            * delta_pose / t_total;
    }

    return velocity_desired;
}
