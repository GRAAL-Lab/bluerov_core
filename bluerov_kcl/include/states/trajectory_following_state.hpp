#pragma once

#include "states/base_auv_state.hpp"

class TrajectoryFollowingState : public BaseAUVState {
private:
    Eigen::Matrix<double, 6, 1> pose_initial_;
    Eigen::Matrix<double, 6, 1> pose_goal_;
    double t_total_;
    double t_current_start_;

public:
    explicit TrajectoryFollowingState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;

    Eigen::Matrix<double, 6, 1> FindNextTrajectoryPoint(
        const Eigen::Matrix<double, 6, 1>& pose_initial,
        const Eigen::Matrix<double, 6, 1>& pose_goal,
        double t_total,
        double t_current) const;
};
