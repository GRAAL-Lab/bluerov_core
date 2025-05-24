#include "states/path_following_state.hpp"


PathFollowingState::PathFollowingState(fsm::FSM* fsm) : BaseAUVState(fsm, "PATH_FOLLOWING") {}

fsm::retval PathFollowingState::OnEntry() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Entering PATH_FOLLOWING state");

    ctrlData->armed_desired = true;
    ctrlData->flightMode_desired = auv_core_helper::FlightMode::GUIDED;
    ctrlData->deisiredCtrlMode = auv_core_helper::BrigdeMode::VelCtrl;

    //update home
    ctrlData->homeGlobal.head<3>() = ctrlData->poseActualGlobal.head<3>();
    
    if (ctrlData->pathPlanningMode == auv_core_helper::PathMode::Serpentine2D) {
        RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Path Planning Serpentine 2D");
        
        if (ctrlData->serpentinePolygonVertices.empty()) {
            return fsm::fail;
        }

        sisl::Path::Direction direction = ctrlData->serpentineDirection
                                            ? sisl::Path::Direction::Backward
                                            : sisl::Path::Direction::Forward;

        path = sisl::PathFactory::NewSerpentine(
            ctrlData->serpentineAngle,
            direction,
            ctrlData->serpentineOffset,
            ctrlData->serpentinePolygonVertices
        );

    } else if (ctrlData->pathPlanningMode == auv_core_helper::PathMode::Spiral2D) {
        RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Path Planning Spiral 2D");

        path = sisl::PathFactory::NewOutwardSpiral(
            Eigen::Vector3d(0, 0, 0),          // centre
            ctrlData->spiralDiameter,          // max diameter (m)
            ctrlData->spiralIncrement          // diameter increment (m)
        );
        std::cout << "Spiral path created with diameter: " << ctrlData->spiralDiameter << " and increment: " << ctrlData->spiralIncrement << std::endl;

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("PathFollowingState"), "Unexpected pathPlanningMode: %s", ctrlData->pathPlanningMode.c_str());
        fsm_->SetNextState(States::HOLD);
        return fsm::fail;
    }


    if (!path) {
        isCurveSet_ = false;
        return fsm::fail;
    }


    // UNCOMMENT THIS BLOCK TO VEIW THE PATH IN RVIZ

    auto sampledPointsSharedPtr = path->Sampling(1500);
    if (!sampledPointsSharedPtr || sampledPointsSharedPtr->empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("PathFollowingState"), "Failed to sample points from SISL path.");
        return fsm::fail;
    }

    nav_msgs::msg::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = rclcpp::Clock().now();

    for (const auto& point : *sampledPointsSharedPtr) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = point.x();
        pose_stamped.pose.position.y = point.y();
        pose_stamped.pose.position.z = point.z();
        pose_stamped.pose.orientation.w = 1.0;
        nav_path.poses.push_back(pose_stamped);
    }
    ctrlData->plannedPath = nav_path;

    // Initialize PID controllers
    ctb::PIDGains gainsX = {ctrlData->gainsX(0), ctrlData->gainsX(1), ctrlData->gainsX(2), ctrlData->gainsX(3), ctrlData->gainsX(4), ctrlData->gainsX(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY(0), ctrlData->gainsY(1), ctrlData->gainsY(2), ctrlData->gainsY(3), ctrlData->gainsY(4), ctrlData->gainsY(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ(0), ctrlData->gainsZ(1), ctrlData->gainsZ(2), ctrlData->gainsZ(3), ctrlData->gainsZ(4), ctrlData->gainsZ(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll(0), ctrlData->gainsRoll(1), ctrlData->gainsRoll(2), ctrlData->gainsRoll(3), ctrlData->gainsRoll(4), ctrlData->gainsRoll(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch(0), ctrlData->gainsPitch(1), ctrlData->gainsPitch(2), ctrlData->gainsPitch(3), ctrlData->gainsPitch(4), ctrlData->gainsPitch(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw(0), ctrlData->gainsYaw(1), ctrlData->gainsYaw(2), ctrlData->gainsYaw(3), ctrlData->gainsYaw(4), ctrlData->gainsYaw(5)};

    pidX_.Initialize(gainsX,    ctrlData->dt, std::max(ctrlData->maxVelocity(0), std::abs(ctrlData->minVelocity(0))));
    pidY_.Initialize(gainsY,    ctrlData->dt, std::max(ctrlData->maxVelocity(1), std::abs(ctrlData->minVelocity(1))));
    pidZ_.Initialize(gainsZ,    ctrlData->dt, std::max(ctrlData->maxVelocity(2), std::abs(ctrlData->minVelocity(2))));
    pidRoll_.Initialize(gainsRoll, ctrlData->dt, std::max(ctrlData->maxVelocity(3), std::abs(ctrlData->minVelocity(3))));
    pidPitch_.Initialize(gainsPitch, ctrlData->dt, std::max(ctrlData->maxVelocity(4), std::abs(ctrlData->minVelocity(4))));
    pidYaw_.Initialize(gainsYaw, ctrlData->dt, std::max(ctrlData->maxVelocity(5), std::abs(ctrlData->minVelocity(5))));

    std::string alosPath_ = ament_index_cpp::get_package_share_directory("auv_core_helper") + "/param/ctrl/alosed_params";
    dynamic_goal_alos::DynamicGoalBasedALOSParams alosParams = dynamic_goal_alos::LoadALOSParamsFromConf(alosPath_);
    alosController_ = std::make_unique<dynamic_goal_alos::DynamicGoalBasedALOS>(alosParams);
    delta_ = alosParams.deltaMax;

    return fsm::ok;
}



fsm::retval PathFollowingState::Execute() noexcept { 
    //print i am alive in this state
    RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Executing PATH_FOLLOWING state");

    static bool initial_time_set = false;
    static rclcpp::Time previous_time;

    if (!isCurveSet_) {
        RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Curve is not set. Waiting for curve to be set.");
        return fsm::ok; 
    }

    while (ctrlData->timeActual.nanoseconds() < 0) {
        RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Waiting for time to be published...");
        return fsm::ok; 
    }

    const auto& poses = ctrlData->plannedPath.poses;

    if (!initial_time_set) {
        previous_time = ctrlData->timeActual;
        initial_time_set = true;
    }

    if (!isVehicleOnPathDirection_) {
        Eigen::Vector3d firstPoint(poses[0].pose.position.x, poses[0].pose.position.y, poses[0].pose.position.z);
        Eigen::Vector3d secondPoint(poses[1].pose.position.x, poses[1].pose.position.y, poses[1].pose.position.z);

        Eigen::Vector3d pathDirection = (secondPoint - firstPoint).normalized();

        double yawDirection = atan2(pathDirection.y(), pathDirection.x());
        double pitchDirection = -atan2(pathDirection.z(), sqrt(pathDirection.x()*pathDirection.x() + pathDirection.y()*pathDirection.y()));

        Eigen::Matrix<double, 6, 1> poseGoalLocal;
        poseGoalLocal << firstPoint.x(), firstPoint.y(), firstPoint.z(), 0, pitchDirection, yawDirection;
        ctrlData->poseGoalLocal = poseGoalLocal;

        // Compute errors (world frame)
        positionXError_ = ctrlData->poseGoalLocal(0) - ctrlData->poseActualLocal(0);
        positionYError_ = ctrlData->poseGoalLocal(1) - ctrlData->poseActualLocal(1);
        positionZError_ = ctrlData->poseGoalLocal(2) - ctrlData->poseActualLocal(2);
        rollError_      =  ctb::AngleDifference(ctrlData->poseGoalLocal(3), ctrlData->poseActualLocal(3));
        pitchError_     =  ctb::AngleDifference(ctrlData->poseGoalLocal(4), ctrlData->poseActualLocal(4));
        yawError_       =  ctb::AngleDifference(ctrlData->poseGoalLocal(5), ctrlData->poseActualLocal(5));

        // Normalize orientation errors for safety
        ctb::NormalizeAngle(rollError_);
        ctb::NormalizeAngle(yawError_);
        ctb::NormalizeAngle(pitchError_);

        ctrlData->velocityDesiredNED(0) = -pidX_.Compute(0, positionXError_);
        ctrlData->velocityDesiredNED(1) = -pidY_.Compute(0, positionYError_);
        ctrlData->velocityDesiredNED(2) = -pidZ_.Compute(0, positionZError_);

        // Compute body-frame angular velocities using PID
        Eigen::Vector3d wDesired = Eigen::Vector3d::Zero();
        wDesired[0] = -pidRoll_.Compute(0, rollError_);
        wDesired[1] = -pidPitch_.Compute(0, pitchError_);
        wDesired[2] = -pidYaw_.Compute(0, yawError_);
        // Directly assign body-frame angular velocities (no Euler angle rate conversion)
        ctrlData->velocityDesiredNED(3) = wDesired[0];
        ctrlData->velocityDesiredNED(4) = wDesired[1];
        ctrlData->velocityDesiredNED(5) = wDesired[2];
        // Check if aligned
        if (std::abs(positionXError_) < 0.1 && std::abs(positionYError_) < 0.1 && std::abs(positionZError_) < 0.1 &&
            std::abs(rollError_) < 0.1 && std::abs(yawError_) < 0.1 && std::abs(pitchError_) < 0.1) {
            RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Vehicle reached starting point and aligned with path direction");
            ctrlData->velocityDesiredNED.setZero();
            isVehicleOnPathDirection_ = true;
        }
    } else {
        double intervalEnd = std::min(currentAbscissa_ + delta_, path->EndParameter());
        closestPointAbscissa_ = path->FindAbscissaClosestPointOnInterval(ctrlData->poseActualLocal.head(3), currentAbscissa_, intervalEnd);
        Eigen::Vector3d currentPoint = path->At(closestPointAbscissa_);
        double goalAbscissa = closestPointAbscissa_ + delta_; 
        goalAbscissa = std::clamp(goalAbscissa, path->StartParameter(), path->EndParameter());
        Eigen::Vector3d nextPoint = path->At(goalAbscissa);
        Eigen::Vector3d direction = (nextPoint - currentPoint).normalized();
        double pi_h = atan2(direction.y(), direction.x());
        double pi_p = -atan2(direction.z(), sqrt(direction.x()*direction.x() + direction.y()*direction.y()));
        double psi_d = pi_h;
        double theta_psi_d = pi_p;
        bool alosSuccess = alosController_->ALOS3D(
            ctrlData->poseActualLocal.head(3),
            nextPoint,
            currentPoint,
            delta_,
            ctrlData->dt,
            theta_psi_d,
            psi_d,
            crossTrackError_,
            verticalTrackError_);

        if (!alosSuccess) {
            RCLCPP_WARN(rclcpp::get_logger("PathFollowingState"), "ALOS3D failed to compute desired heading/pitch.");
            return fsm::ok;
        }
        RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "here 1");

        ctrlData->poseGoalLocal(0) = nextPoint.x();
        ctrlData->poseGoalLocal(1) = nextPoint.y();
        ctrlData->poseGoalLocal(2) = nextPoint.z();
        ctrlData->poseGoalLocal(3) = 0;
        ctrlData->poseGoalLocal(4) = theta_psi_d; 
        ctrlData->poseGoalLocal(5) = psi_d;
        // Compute errors in world frame
        positionXError_ = ctrlData->poseGoalLocal(0) - ctrlData->poseActualLocal(0);
        positionYError_ = ctrlData->poseGoalLocal(1) - ctrlData->poseActualLocal(1);
        positionZError_ = ctrlData->poseGoalLocal(2) - ctrlData->poseActualLocal(2);
        rollError_      =  ctb::AngleDifference(ctrlData->poseGoalLocal(3), ctrlData->poseActualLocal(3));
        pitchError_     =  ctb::AngleDifference(ctrlData->poseGoalLocal(4), ctrlData->poseActualLocal(4));
        yawError_       =  ctb::AngleDifference(ctrlData->poseGoalLocal(5), ctrlData->poseActualLocal(5));

        ctb::NormalizeAngle(rollError_);
        ctb::NormalizeAngle(yawError_);
        ctb::NormalizeAngle(pitchError_);

        // PID on body-frame linear errors
        ctrlData->velocityDesiredNED(0) = -pidX_.Compute(0, positionXError_);
        ctrlData->velocityDesiredNED(1) = -pidY_.Compute(0, positionYError_);
        ctrlData->velocityDesiredNED(2) = -pidZ_.Compute(0, positionZError_);

        // PID on angular errors for body-frame angular velocities
        Eigen::Vector3d wDesired = Eigen::Vector3d::Zero();
        wDesired[0] = -pidRoll_.Compute(0, rollError_);
        wDesired[1] = -pidPitch_.Compute(0, pitchError_);
        wDesired[2] = -pidYaw_.Compute(0, yawError_);
        // Assign body-frame angular velocities directly
        ctrlData->velocityDesiredNED(3) = wDesired[0];
        ctrlData->velocityDesiredNED(4) = wDesired[1];
        ctrlData->velocityDesiredNED(5) = wDesired[2];
        Eigen::Vector3d currentPosDot = path->Derivate(1, closestPointAbscissa_).front();
        Eigen::Vector3d goalPosDot = path->Derivate(1, goalAbscissa).front();
        Eigen::Vector3d currentDirection = currentPosDot.normalized();
        Eigen::Vector3d goalDirection = goalPosDot.normalized();
        double tangentsDifferenceNorm = (goalDirection - currentDirection).norm();
        delta_ = alosController_->UpdateLookAheadDistance(crossTrackError_, verticalTrackError_, tangentsDifferenceNorm);
        double path_completed = (closestPointAbscissa_ / path->EndParameter()) * 100.0;
        //print cte, vte, delta, time, path_completed in diffrent lines
        // RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Cross-track error: %f", crossTrackError_);
        // RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Vertical-track error: %f", verticalTrackError_);
        // RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Delta: %f", delta_);
        // RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Time: %f", ctrlData->timeActual.seconds());
        // RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Path completed: %f", path_completed);
        RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "here 1");

        currentAbscissa_ = closestPointAbscissa_;
        if (path_completed >= 99.95) {
            RCLCPP_INFO(rclcpp::get_logger("PathFollowingState"), "Path has ended");
            fsm_->SetNextState(States::HOLD);
            return fsm::ok;
        }
    }
    return fsm::ok;
}


fsm::retval PathFollowingState::OnExit() noexcept {
    ctrlData->plannedPath.poses.clear();
    isVehicleOnPathDirection_ = false;
    closestPointAbscissa_ = 0.0;
    currentAbscissa_ = 0.0;
    alosController_.reset();
    return fsm::ok;
}
