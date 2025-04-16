#pragma once

#include "states/base_auv_state.hpp"
#include <sisl_toolbox/sisl_toolbox.hpp>
#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <filesystem>
#include <limits>
#include <auv_core_helper/path_modes.hpp>
#include <dynamic_goal_ALOS.hpp>


/// The `PathFollowingState` class manages the autonomous path following behavior of the AUV.
/// It supports various path types (e.g., serpentine, helical) and uses PID controllers for guidance.
class PathFollowingState : public BaseAUVState {
private:
    // Path-related variables
    std::shared_ptr<sisl::Path> path; ///< SISL-generated path.
    std::vector<Eigen::Vector3d> sampledPoints; ///< Sampled points from the path.
    bool isCurveSet_ = false; ///< Whether the path is set.
    bool isVehicleOnPathDirection_ = false; ///< Whether the vehicle is aligned with the path direction.
    double currentAbscissa_ = 0.0; ///< Current abscissa (progress along the path).
    double closestPointAbscissa_ = 0.0; ///< Abscissa of the closest point on the path.

    

    // Error metrics
    double positionXError_ = 0.0; ///< Error in the X position.
    double positionYError_ = 0.0; ///< Error in the Y position.
    double positionZError_ = 0.0; ///< Error in the Z position.
    double rollError_ = 0.0; ///< Error in the roll orientation.
    double yawError_ = 0.0; ///< Error in the yaw orientation.
    double pitchError_ = 0.0; ///< Error in the pitch orientation.
    double crossTrackError_ = 0.0; ///< Cross-track error.
    double verticalTrackError_ = 0.0; ///< Vertical track error.

    // PID controllers
    ctb::DigitalPID pidX_, pidY_, pidZ_; ///< Position PIDs.
    ctb::DigitalPID pidRoll_, pidPitch_, pidYaw_; ///< Orientation PIDs.

    

    // Constants
    static constexpr double MAX_PITCH = 0.610865; ///< Maximum pitch angle (±35 degrees) in radians.
    static constexpr double MIN_PITCH = -0.610865; ///< Minimum pitch angle (±35 degrees) in radians.


    std::unique_ptr<dynamic_goal_alos::DynamicGoalBasedALOS> alosController_;
    double delta_;

public:
    /// Constructor
    explicit PathFollowingState(fsm::FSM* fsm);

    /// Called when entering the path following state.
    fsm::retval OnEntry() noexcept override;

    /// Called repeatedly during the path following state.
    fsm::retval Execute() noexcept override;

    /// Called when exiting the path following state.
    fsm::retval OnExit() noexcept override;
};
