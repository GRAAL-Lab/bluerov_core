#include "mission_ctrl/mission_controller.hpp"

namespace mission {
MissionController::MissionController()
    : Node("mission_control_node")
{
    ctrlData_ = std::make_shared<ControlData>();
    systemStatus_ = std::make_shared<SystemStatus>(this->get_clock()->get_clock_type());
    LoadConfiguration(); // REQUIRES SYSTEM STATUS TO BE INITIALIZED

    SetUpFSM();

    debugPub_ = this->create_publisher<std_msgs::msg::String>(
        "/auv/mission/debug", rclcpp::SystemDefaultsQoS());

    missionStatusPub_ = this->create_publisher<auv_core_helper::msg::MissionStatus>(
        auv_core_helper::topicnames::mission_status, rclcpp::SystemDefaultsQoS());

    systemStatusSub_ = this->create_subscription<auv_core_helper::msg::SystemStatus>(
        auv_core_helper::topicnames::system_status, rclcpp::SystemDefaultsQoS(),
        std::bind(&mission::MissionController::SystemStatusCB, this, std::placeholders::_1));

    setKCLClient_ = rclcpp_action::create_client<auv_core_helper::action::SetKCL>(
        this, auv_core_helper::topicnames::kcl_setter_action);

    perceptionSub_ = this->create_subscription<auv_core_helper::msg::DtcList>(
        auv_core_helper::topicnames::objects, rclcpp::SystemDefaultsQoS(),
        std::bind(&MissionController::PerceptionCB, this, std::placeholders::_1));

    KclSub_ = this->create_subscription<auv_core_helper::msg::KclStatus>(
        auv_core_helper::topicnames::kcl_state, rclcpp::SystemDefaultsQoS(),
        std::bind(&MissionController::KclCB, this, std::placeholders::_1));

    poseSub_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_actual_global_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&MissionController::PoseCB, this, std::placeholders::_1));

    missionCommandService_ = this->create_service<auv_core_helper::srv::MissionCommand>(
        auv_core_helper::topicnames::mission_cmd_service,
        std::bind(&MissionController::MissionCommandCB, this, std::placeholders::_1, std::placeholders::_2));

    setGimbalAttitudeService_ = this->create_client<auv_core_helper::srv::SetGimbalAttitude>(
        auv_core_helper::topicnames::gimbal_service);

    kclSendGoalOptions_ = rclcpp_action::Client<auv_core_helper::action::SetKCL>::SendGoalOptions();
    kclSendGoalOptions_.result_callback = std::bind(&MissionController::ActionResultCallback, this, std::placeholders::_1);
    kclSendGoalOptions_.feedback_callback = std::bind(&MissionController::ActionFeedbackCallback, this,
        std::placeholders::_1, std::placeholders::_2);

    int msRunPeriod = 1.0 / (systemStatus_->conf.ctrlRate) * 1000;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&MissionController::Run, this));

    if (systemStatus_->conf.simCtrlStation) {
        simCtrlStationTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000), std::bind(&MissionController::SimulateMissionCmdFromFile, this));
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "[DEBUG SETTING] --> No control station");
    } else {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Waiting for task data to be set by ctrl station");
    }

    if (systemStatus_->conf.simBridge)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No bridge");
    if (systemStatus_->conf.simKcl)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No KCL");
    if (systemStatus_->conf.simPerception)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No Perception");

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Waiting for components to be alive...");
};

void MissionController::StatusPub()
{
    auv_core_helper::msg::MissionStatus status;
    status.stamp = this->get_clock()->now();
    if (systemStatus_->State() == MissionCtrlState::WAITING_FOR_SYSTEM_TO_BE_READY) {
        status.state = states::ID::init;
        status.task_benchmark = "WAITING FOR SYSTEM";
    } else if (systemStatus_->State() == MissionCtrlState::WAITING_FOR_MISSION_CMD) {
        status.state = states::ID::init;
        status.task_benchmark = "WAITING FOR CMD";
    } else {
        if (taskData_ == nullptr || taskData_->taskPhases.empty()) {
            status.task_benchmark = "RECEIVED A CMD";
        } else {
            status.task_benchmark = taskData_->taskType;
            status.state = taskData_->taskPhases.front().first;
            status.state_object = taskData_->taskPhases.front().second;
        }
    }
    status.requests.obstacles = ctrlData_->perceptionData.enableDtcObstacles;
    status.requests.buoys = ctrlData_->perceptionData.enableDtcBuoys;
    status.requests.main_pipe = ctrlData_->perceptionData.enableDtcMainPipe;
    status.requests.manipulation_console = ctrlData_->perceptionData.enableDtcManipulationConsole;

    missionStatusPub_->publish(status);
}

void MissionController::Run()
{
    debugMsg.data = "time since last feedback: " + std::to_string((this->get_clock()->now() - systemStatus_->lastKclFeedbackTime).seconds()) + " s";
    debugPub_->publish(debugMsg);

    std::cerr << "in Run()\n";

    systemStatus_->CheckSystemLiveness(this->get_clock()->now());

    // Publish status
    StatusPub();

    //=== Check if the system is ready to run ===
    if (systemStatus_->State() == MissionCtrlState::WAITING_FOR_SYSTEM_TO_BE_READY) {
        // Wait for system to be alive
        // ResetTaskDataFSM();
        // rFsm_.SetInitState(states::ID::init);
        return;
    }

    //=== Check the FSM ===
    auto now = this->get_clock()->now();
    if (rFsm_.GetCurrentStateName() != rFsm_.GetNextStateName()) {
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "FSM switched to state "<< rFsm_.GetNextStateName().c_str());
        systemStatus_->lastStateSwitchTime = now;

        kclCancelCmd();
        // if (rFsm_.GetNextStateName() == states::ID::init) {
        //     // Back to initial state
        //     // systemStatus_->SetState(MissionCtrlState::WAITING_FOR_MISSION_CMD); DONE INTO STATE BASE
        //     ResetTaskDataFSM();
        // }

    } else {
        if (systemStatus_->State() == MissionCtrlState::WAITING_FOR_MISSION_CMD) {
            systemStatus_->lastStateSwitchTime = now;
        }
        double timeSinceLastSwitch = now.seconds() - systemStatus_->lastStateSwitchTime.seconds();
        // Get current state timeout value TODO if same state with different objectives than timeout is not gonna reset
        double currentStateTimeout = statesMap_[rFsm_.GetCurrentStateName()]->stateTimeout;
        if (timeSinceLastSwitch > currentStateTimeout && std::fmod(timeSinceLastSwitch, 5.0) < 1.0) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "FSM in state" << rFsm_.GetCurrentStateName().c_str() << "for"<< (int)timeSinceLastSwitch<<"seconds");
        }
    }

    //=== Progress execution ===
    // Switch State (if something happens)
    rFsm_.SwitchState();
    // Process Events
    rFsm_.ProcessEventQueue();
    // Execute current state
    rFsm_.ExecuteState();

    if (systemStatus_->State() != MissionCtrlState::ON_A_MISSION) {
        // Wait for cmd
        // kclCancelCmd();
        return;
    }

    SetGimbalAttitude();

    if (ctrlData_->kclData.kclActionCmd.newCmd) {
        kclCmd();
    }

    auto timeSinceLastKclFeedback = (this->get_clock()->now() - systemStatus_->lastKclFeedbackTime).seconds();
    if (ctrlData_->kclData.kclActionCmd.underExecution && timeSinceLastKclFeedback > 2) {
        ctrlData_->kclData.kclActionCmd.underExecution = false;
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "     -------     KCL command is not under execution     -------     ");
        kclCmd();
    }

    // if (taskData_->taskPhases.empty()) {
    //     ResetTaskDataFSM();
    // }
};

void MissionController::SystemStatusCB(const auv_core_helper::msg::SystemStatus::SharedPtr msg)
{
    bool bridgeAlive = systemStatus_->conf.simBridge || msg->bridge;
    bool kclAlive = systemStatus_->conf.simKcl || msg->kcl;
    bool perceptionAlive = systemStatus_->conf.simPerception || msg->perception;
    bool vehicleIsSafe = msg->vehicle_is_armed;
    systemStatus_->lastSystemStatusTime = this->get_clock()->now();

    bool systemOperational = bridgeAlive && kclAlive && perceptionAlive && vehicleIsSafe;

    if (systemOperational) {
        if (systemStatus_->State() == MissionCtrlState::WAITING_FOR_SYSTEM_TO_BE_READY) {
            systemStatus_->SetStateReady();
            rFsm_.SetInitState(states::ID::init); // Reset FSM to init state
            ResetTaskDataFSM();
            if (systemStatus_->conf.simCtrlStation) {
                simCtrlStationTimer_->reset();
            }
            ctrlData_->perceptionData.desiredGimbalAttitude = 0.0;
        }
    } else {
        systemStatus_->SetState(MissionCtrlState::WAITING_FOR_SYSTEM_TO_BE_READY);
    }
}

void MissionController::PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg)
{

    ctrlData_->perceptionData.newDtcFromPerception = true;
    ctrlData_->perceptionData.dtcList = *msg;

    for (auto& buoy : msg->buoys) {
        Buoy b;
        b.detectionId = buoy.id;
        b.position.latitude = buoy.position.latitude;
        b.position.longitude = buoy.position.longitude;
        b.radius = buoy.radius;
        b.color = buoy.color;
        b.colorConfidence = buoy.color_confidence;

        ctrlData_->perceptionData.detectedBuoys[buoy.id] = b;
        // std::cerr << "Detected buoy: " << buoy.id << " at position: ["
        //           << buoy.position.latitude << ", " << buoy.position.longitude << "]\n";

        if (systemStatus_->conf.debugBuoys) {
            // find the closest true postion and compute error
            ctb::LatLong closestTruePosition;
            double minDistance = std::numeric_limits<double>::max();
            for (const auto& gt_buoy : systemStatus_->conf.debugBuoysPositions) {
                double azimuth, distance;
                ctb::DistanceAndAzimuthRad(b.position, gt_buoy, distance, azimuth);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestTruePosition = gt_buoy;
                }
            }
            // std::cerr << "Closest true position: [" << closestTruePosition.latitude << ", "
            //           << closestTruePosition.longitude << "] with distance: " << minDistance << "\n";
        }
    }
}

void MissionController::KclCB(const auv_core_helper::msg::KclStatus::SharedPtr msg)
{
    (void)msg;
    // systemStatus_->lastKCLTime = this->get_clock()->now();
}

void MissionController::MissionCommandCB(
    const std::shared_ptr<auv_core_helper::srv::MissionCommand::Request> request,
    std::shared_ptr<auv_core_helper::srv::MissionCommand::Response> response)
{
    // if(systemStatus_->conf.restartLatestMission){
    //     latestMissionCmdRequest_ = std::make_shared<auv_core_helper::srv::MissionCommand::Request>(*request);
    //     latestMissionCmdSet_ = true;
    // }

    if (!systemStatus_->SetState(MissionCtrlState::ON_A_MISSION)) {
        response->res = false;
        response->text = "Mission controller is not ready to accept commands.";
        RCLCPP_ERROR(this->get_logger(), "%s", response->text.c_str());
        return;
    }

    if (request->tbm_id == 1) {
        taskData_ = std::make_shared<Inspection>();
    } else if (request->tbm_id == 2) {
        taskData_ = std::make_shared<Intervention>();
    } else if (request->tbm_id == 3) {
        taskData_ = std::make_shared<InspectionAndIntervention>();
    } else {
        response->res = false;
        response->text = "Invalid tbm id: " + std::to_string(request->tbm_id);
        return;
    }

    if (!taskData_->ConfigureFromSrv(request)) {
        response->res = false;
        response->text = "Failed to configure task data from request";
        return;
    }

    // for (const auto& point : taskData_->buoysArea.points) {
    //     if (!IsPointWithinBoundaries(point)) {
    //         response->res = false;
    //         response->text = "Buoys area point is outside of safety boundaries: [" + std::to_string(point.latitude) + ", " + std::to_string(point.longitude) + "]";
    //         RCLCPP_ERROR(this->get_logger(), "%s", response->text.c_str());
    //         return;
    //     }
    // }

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Mission command received, tbm_id: "<< request->tbm_id);
    response->res = StartMission();
}

bool MissionController::StartMission()
{
    // start mission shares the taskdata among the states
    // put the FSM in the init state
    // set the home position as the current position
    // set the surface depth as the current depth if it is less than 1.0

    // if (taskData_ == nullptr) {
    //     RCLCPP_ERROR(this->get_logger(), "Cannot start mission, task data is not set.");
    //     return false;
    // }
    // if (!systemStatus_->IsSystemAlive(this->get_clock()->now())) {
    //     RCLCPP_ERROR(this->get_logger(), "Cannot start mission, system is not alive.");
    //     return false;
    // }

    SetTaskDataFSM();

    // systemStatus_->missionUnderExecution = true;

    // rFsm_.SetInitState(mission::states::ID::init); // Reset FSM to init state

    // systemStatus_->timeOutsideSafetyArea = this->get_clock()->now();

    stateHoming_->homePosition.latitude = ctrlData_->inertialF_linearPosition.latitude;
    stateHoming_->homePosition.longitude = ctrlData_->inertialF_linearPosition.longitude;

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Set home position as current one: ["<<stateHoming_->homePosition.latitude<<","<<stateHoming_->homePosition.longitude<<"]");

    if (systemStatus_->conf.useStartingDepthAsSurfaceDepth && ctrlData_->depth < 1.0) {
        systemStatus_->conf.surfaceDepth = ctrlData_->depth;
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Setting surface depth as current depth: ["<<systemStatus_->conf.surfaceDepth<<"]");
    }

    std::stringstream ss;
    ss << *taskData_;
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, " ===== TaskBenchMark  "<<taskData_->taskType.c_str()<<"=====");
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, ""<< ss.str().c_str());
    return true;
}

void MissionController::PoseCB(const auv_core_helper::msg::PoseStamped::SharedPtr msg)
{
    // systemStatus_->readFirstPose = true;
    ctrlData_->inertialF_linearPosition.latitude = msg->position.latitude;
    ctrlData_->inertialF_linearPosition.longitude = msg->position.longitude;
    ctrlData_->depth = msg->depth;
    ctrlData_->bodyF_angularPosition.RPY(msg->roll,
        msg->pitch, msg->yaw);

    if (systemStatus_->conf.simCtrlStation && !stateHoming_->homePositionSet) {
        stateHoming_->homePositionSet = true;
        stateHoming_->homePosition.latitude = 44.09595617190768;
        stateHoming_->homePosition.longitude = 9.864626568817506;
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "[DEBUG SETTING] --> Simulating control station, setting home position as the one in front of the dock:["<<stateHoming_->homePosition.latitude<<","<< stateHoming_->homePosition.longitude<<"]");
    }

    // if (!systemStatus_->missionUnderExecution)
    //     return;

    // // Check if the vehicle has finally reached the safety area
    // if (!systemStatus_->vehicleReachedSafetyArea) {
    //     if (IsPointWithinBoundaries(ctrlData_->inertialF_linearPosition)) {
    //         systemStatus_->vehicleReachedSafetyArea = true;
    //         RCLCPP_INFO(this->get_logger(), "Vehicle reached safety area!");
    //     } else {
    //         if (systemStatus_->IsSafetyAreaBreach(this->get_clock()->now())) {
    //             RCLCPP_ERROR(this->get_logger(), "Vehicle did not reach the safety area soon enough.");
    //             kclStopCmd();
    //         }
    //     }
    // } else {
    //     if (!IsPointWithinBoundaries(ctrlData_->inertialF_linearPosition)) {
    //         RCLCPP_ERROR(this->get_logger(), "Vehicle exited the safety area.");
    //         kclStopCmd();
    //     }
    // }
}

// bool MissionController::IsPointWithinBoundaries(const ctb::LatLong& point)
// {
//     if (systemStatus_->conf.safetyBoundary.size() < 3) {
//         throw std::runtime_error("Safety boundary must have at least 3 points.");
//     }

//     std::vector<Eigen::Vector3d> polygon;
//     for (auto& latlong : systemStatus_->conf.safetyBoundary) {
//         Eigen::Vector3d bodyF_point;
//         ctb::LatLong2LocalNED(latlong, 0, point, bodyF_point);
//         polygon.push_back(bodyF_point);
//     }

//     bool pos = false, neg = false;
//     int i = 0;
//     for (auto point : polygon) {
//         auto nextPoint = polygon[(i + 1) % polygon.size()];
//         double cross = (nextPoint.x() - point.x()) * (-point.y()) - (nextPoint.y() - point.y()) * (-point.x());
//         if (cross < 0)
//             neg = true;
//         if (cross > 0)
//             pos = true;
//         if (pos && neg)
//             return false; // point is outside
//         i++;
//     }

//     return true;
// }

bool MissionController::kclCancelCmd()
{
    setKCLClient_->async_cancel_all_goals();
    ctrlData_->kclData.kclActionCmd.underExecution = false;
    return true;
}

bool MissionController::kclStopCmd()
{
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Sending IDLE cmd and moving to INIT state.");

    auv_core_helper::action::SetKCL::Goal goal;
    goal.desired_state = "IDLE";
    // while (!setKCLClient_->wait_for_action_server(std::chrono::seconds(1))) {
    //     RCLCPP_WARN(this->get_logger(), "Waiting for KCL action server to be ready...");
    // }
    setKCLClient_->async_send_goal(goal, kclSendGoalOptions_);
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "KCL action server IDLE cmd sent.");

    ResetTaskDataFSM();
    return true;
}

bool MissionController::kclCmd()
{
    kclCancelCmd();

    auv_core_helper::action::SetKCL::Goal goal;
    goal = ctrlData_->kclData.kclActionCmd.goal;

    if (systemStatus_->conf.debugPrints) {
        if (goal.desired_state == "WAYPOINT_NAVIGATION") {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Sending command to KCL ["<<goal.desired_state.c_str()<<"], "<<goal.position.latitude<<","<< goal.position.longitude<<","<< goal.depth);
        } else {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Sending command to KCL ["<<goal.desired_state.c_str()<<"]");
        }
    }

    if (systemStatus_->conf.simKcl) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Received cmd for KCL but simulating it, so setting it as completed.");
        if (goal.desired_state == "WAYPOINT_NAVIGATION") {
            ctrlData_->inertialF_linearPosition.latitude = goal.position.latitude;
            ctrlData_->inertialF_linearPosition.longitude = goal.position.longitude;
            ctrlData_->depth = goal.depth;
        }
        return true;
    }
    auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(0.5));
    if (setKCLClient_->wait_for_action_server(timeout)) {
        setKCLClient_->async_send_goal(goal, kclSendGoalOptions_);
    } else {
        // systemStatus_->kclActionServerAlive = false;
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "KCL is not ready to rcv a cmd.");
        return false;
    }
    ctrlData_->kclData.kclActionCmd.newCmd = false;
    ctrlData_->kclData.kclActionCmd.underExecution = true;
    systemStatus_->lastKclFeedbackTime = this->get_clock()->now();

    return true;
}

void MissionController::ActionResultCallback(const rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::WrappedResult& result)
{
    ctrlData_->kclData.kclActionCmd.result.success = result.result->success;
    ctrlData_->kclData.kclActionCmd.result.message = result.result->message;
    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(),
        *get_clock(),
        1000,
        "KCL command RESULT ["
        << (ctrlData_->kclData.kclActionCmd.result.success ? "SUCCESS" : "FAILURE")
        << "] with message ["
        << ctrlData_->kclData.kclActionCmd.result.message.c_str()
        << "]"
    );
    // ctrlData_->kclData.kclActionCmd.underExecution = false;
    //  if (result.result->success) {
    //      ctrlData_->kclData.kclActionCmd.completed = false;
    //  }
}

void MissionController::ActionFeedbackCallback(
    rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::SharedPtr,
    const std::shared_ptr<const auv_core_helper::action::SetKCL::Feedback>& feedback)
{
    ctrlData_->kclData.kclActionCmd.feedback.actual_state = feedback->actual_state;
    ctrlData_->kclData.kclActionCmd.feedback.action_progress = feedback->action_progress;
    systemStatus_->lastKclFeedbackTime = this->get_clock()->now();

    if (feedback->actual_state == "PATH_FOLLOWING" && feedback->action_progress >= 95.0) {
        // probably we are doing a search path and it is almost done without founding the goal so stop the mission
        RCLCPP_ERROR(this->get_logger(), "KCL almost done with path following, but not finding the goal, so stopping the mission.");
        kclStopCmd();
        
    }

    if (ctrlData_->kclData.kclActionCmd.underExecution && ctrlData_->kclData.kclActionCmd.goal.desired_state != ctrlData_->kclData.kclActionCmd.feedback.actual_state) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Cancelling KCL action since actual state is different from the desired state: ["<<ctrlData_->kclData.kclActionCmd.goal.desired_state.c_str()<<"] vs ["<<ctrlData_->kclData.kclActionCmd.feedback.actual_state.c_str()<<"]");
        kclCancelCmd();
        ctrlData_->kclData.kclActionCmd.underExecution = true; // This will trigger a new command in the next run
    }

    if (systemStatus_->conf.debugPrints) {
        if (std::fmod(ctrlData_->kclData.kclActionCmd.feedback.action_progress, 10.0) < 1.0) {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "KCL command FEEDBACK: ["<<ctrlData_->kclData.kclActionCmd.feedback.actual_state.c_str()<<"] with progress "<<ctrlData_->kclData.kclActionCmd.feedback.action_progress);
        }
    }
}

void MissionController::SetGimbalAttitude()
{
    if (ctrlData_->perceptionData.desiredGimbalAttitude != lastSetGimbalAttitude_) {
        auto timeoutMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(0.250)); // 250 ms

        std::cerr << "Setting gimbal attitude to: " << ctrlData_->perceptionData.desiredGimbalAttitude << std::endl;

        if (setGimbalAttitudeService_->service_is_ready()) {
            auto request = std::make_shared<auv_core_helper::srv::SetGimbalAttitude::Request>();
            request->yaw = ctrlData_->perceptionData.desiredGimbalAttitude;

            auto future = setGimbalAttitudeService_->async_send_request(request);

            // Wait up to timeoutMilliseconds for the result
            auto ret = rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                future,
                timeoutMilliseconds);

            if (ret == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                if (response->success) {
                    lastSetGimbalAttitude_ = ctrlData_->perceptionData.desiredGimbalAttitude;
                    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Gimbal attitude set successfully.");
                } else {
                    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Service call failed to set gimbal attitude.");
                }
            } else {
                RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Timed out waiting for gimbal attitude service response.");
            }
        } else {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),*get_clock(),1000, "Gimbal attitude service is not ready.");
        }
    }
}

void MissionController::LoadConfiguration()
{
    // Load configuration from file
    // std::string package_share_directory = ament_index_cpp::get_package_share_directory("mission_ctrl");
    // std::string confPath = package_share_directory + "/conf/" + "system.conf";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("auv_core_helper");
    std::string confPath = package_share_directory + "/param/ctrl/" + "mission_ctrl.conf";
    libconfig::Config confObj;

    try {
        confObj.readFile(confPath.c_str());
        ctb::GetParam(confObj, systemStatus_->conf.simKcl, "simulate_kcl");
        ctb::GetParam(confObj, systemStatus_->conf.simPerception, "simulate_perception");
        ctb::GetParam(confObj, systemStatus_->conf.simBridge, "simulate_bridge");
        ctb::GetParam(confObj, systemStatus_->conf.simCtrlStation, "simulate_ctrl_station");
        ctb::GetParam(confObj, systemStatus_->conf.debugPrints, "debug_prints");

        // ctb::GetParam(confObj, systemStatus_->conf.restartLatestMission, "restart_latest_mission");
        ctb::GetParam(confObj, systemStatus_->conf.useStartingDepthAsSurfaceDepth, "use_starting_depth_as_surface_depth");
        ctb::GetParam(confObj, systemStatus_->conf.surfaceDepth, "surface_depth");
        ctb::GetParam(confObj, systemStatus_->conf.diveDepth, "dive_depth");
        ctb::GetParam(confObj, systemStatus_->conf.depthTolerance, "depth_tolerance");
        ctb::GetParam(confObj, systemStatus_->conf.latlongTolerance, "latlong_tolerance");
        ctb::GetParam(confObj, systemStatus_->conf.ctrlRate, "ctrl_rate");

        ctb::GetParam(confObj, systemStatus_->conf.debugBuoys, "buoysDebug");
        ctb::GetParam(confObj, systemStatus_->conf.ignoreBuoyColor, "ignore_buoy_color");
        ctb::GetParam(confObj, systemStatus_->conf.gateWpsDistance, "gate_wps_distance");
        

        ctb::GetParam(confObj, systemStatus_->conf.initStateTimeout, "init_state_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.homingStateTimeout, "homing_state_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.moveToWpStateTimeout, "move_to_wp_state_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.searchForObjectStateTimeout, "search_for_object_state_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.crossGateStateTimeout,"cross_gate_state_timeout" );
        ctb::GetParam(confObj, systemStatus_->conf.searchBuoyAreaStateTimeout,"search_buoy_area_state_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.inspectBuoyStateTimeout, "inspect_buoy_state_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.inspectPipesStateTimeout, "inspect_pipe_state_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.updateLocalizationStateTimeout, "update_localization_state_timeout");


        const libconfig::Setting& root = confObj.getRoot();

        // debug_positions
        ctb::GetParam(confObj, systemStatus_->conf.debug_position_selection, "debug_position_selection");
        std::cerr << "Debug positions (selected " << systemStatus_->conf.debug_position_selection << "):" << std::endl;
        const libconfig::Setting& debugPositionsSetting = root["debug_positions"];
        for (int i = 0; i < debugPositionsSetting.getLength(); ++i) {
            const libconfig::Setting& point = debugPositionsSetting[i];
            Eigen::VectorXd localTmp;
            ctb::GetParamVector(point, localTmp, "point");

            ctb::LatLong latLongTmp(localTmp[0], localTmp[1]);
            systemStatus_->conf.debugPositions.push_back(latLongTmp);
        }

        // Eigen::VectorXd latLongTmp;
        // ctb::GetParamVector(confObj, latLongTmp, "debug_position");
        // systemStatus_->conf.debugPosition.latitude = latLongTmp[0];
        // systemStatus_->conf.debugPosition.longitude = latLongTmp[1];
        // if (systemStatus_->conf.simBridge) {
        //     ctrlData_->inertialF_linearPosition = systemStatus_->conf.debugPosition;
        //}

        if (systemStatus_->conf.debugBuoys) {
            std::cerr << "Debug buoys positions:" << std::endl;
            std::cerr << std::setprecision(15);
            const libconfig::Setting& buoysSetting = root["buoysStonefishPositions"];
            for (int i = 0; i < buoysSetting.getLength(); ++i) {
                const libconfig::Setting& point = buoysSetting[i];
                Eigen::VectorXd localTmp;
                ctb::GetParamVector(point, localTmp, "point");
                ctb::LatLong stonefishCentroid(44.095952330602564, 9.865115308770484); // from update pose stonefish in stonefish utils
                ctb::LatLong latLongTmp;
                double alt;
                Eigen::Vector3d localTmp3d;
                localTmp3d << localTmp[0], localTmp[1], 0.0; // Assuming the z-coordinate is 0 for the buoy positions

                double theta = 1.85;
                Eigen::Matrix3d R_offset;
                R_offset << std::cos(theta), -std::sin(theta), 0.0,
                    std::sin(theta), std::cos(theta), 0.0,
                    0.0, 0.0, 1.0;
                localTmp3d = R_offset.transpose() * localTmp3d;

                ctb::LocalNED2LatLong(localTmp3d, stonefishCentroid, latLongTmp, alt);
                std::cerr << "  - stonefish position: " << localTmp[0] << ", " << localTmp[1] << std::endl;
                std::cerr << "  - ned local: [" << localTmp3d[0] << ", " << localTmp3d[1] << ", " << localTmp3d[2] << "]" << std::endl;
                std::cerr << "  - latlong: [" << latLongTmp.latitude << ", " << latLongTmp.longitude << "]" << std::endl;
                std::cerr << " --  " << std::endl;
                systemStatus_->conf.debugBuoysPositions.push_back(latLongTmp);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Configuration loaded from file: %s", confPath.c_str());
        RCLCPP_INFO(this->get_logger(), "simKcl: %d", systemStatus_->conf.simKcl);
        RCLCPP_INFO(this->get_logger(), "simPerception: %d", systemStatus_->conf.simPerception);
        RCLCPP_INFO(this->get_logger(), "simBridge: %d", systemStatus_->conf.simBridge);
        RCLCPP_INFO(this->get_logger(), "simCtrlStation: %d", systemStatus_->conf.simCtrlStation);
        RCLCPP_INFO(this->get_logger(), "debugPrints: %d", systemStatus_->conf.debugPrints);
        RCLCPP_INFO(this->get_logger(), "depthTolerance: %f", systemStatus_->conf.depthTolerance);
        RCLCPP_INFO(this->get_logger(), "latlongTolerance: %f", systemStatus_->conf.latlongTolerance);
        RCLCPP_INFO(this->get_logger(), "ctrlRate: %i", systemStatus_->conf.ctrlRate);
        RCLCPP_INFO(this->get_logger(), "localizationTimeout: %f", systemStatus_->conf.localizationTimeout);
        RCLCPP_INFO(this->get_logger(), "Safety boundary points:");
        for (const auto& point : systemStatus_->conf.safetyBoundary) {
            RCLCPP_INFO(this->get_logger(), "  - [%f, %f]", point.latitude, point.longitude);
        }

    } catch (const libconfig::FileIOException& fioex) {
        RCLCPP_ERROR(this->get_logger(), "I/O error while reading file: %s", fioex.what());
        RCLCPP_ERROR(this->get_logger(), "  Path: '%s'. Make sure the file exists and is readable.", confPath.c_str());
    } catch (const libconfig::ParseException& pex) {
        RCLCPP_ERROR(this->get_logger(), "Parse error at %s:%d - %s", pex.getFile(), pex.getLine(), pex.getError());
    }
}

void MissionController::SimulateMissionCmdFromFile()
{

    if (!systemStatus_->SetState(MissionCtrlState::ON_A_MISSION)) {
        RCLCPP_ERROR(this->get_logger(), "Mission controller is not ready to accept commands.");
        return;
    }

    libconfig::Config confObj;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("mission_ctrl");
    std::string confPath = package_share_directory + "/conf/" + "tasks.conf";

    try {
        confObj.readFile(confPath.c_str());
        uint tbm_id;
        ctb::GetParam(confObj, tbm_id, "tbm");

        switch (tbm_id) {
        case 1:
            taskData_ = std::make_shared<Inspection>();
            break;
        case 2:
            taskData_ = std::make_shared<Intervention>();
            break;
        case 3:
            taskData_ = std::make_shared<InspectionAndIntervention>();
            break;
        default:
            std::cerr << "Invalid tbm id: " << tbm_id << std::endl;
        }
        taskData_->ConfigureFromFile(confObj);
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        std::cerr << "  Path: '" << confPath << "'. Make sure the file exists and is readable." << std::endl;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
    }

    if (StartMission())
        simCtrlStationTimer_->cancel();
}

void MissionController::SetUpFSM()
{
    stateInit_ = std::make_shared<states::StateInit>();
    stateHoming_ = std::make_shared<states::StateHoming>();
    stateLatLong_ = std::make_shared<states::StateLatLong>();
    stateSearchObject_ = std::make_shared<states::StateSearchObject>();
    stateCrossGate_ = std::make_shared<states::StateCrossGate>();
    stateSearchBuoyArea_ = std::make_shared<states::StateSearchBuoyArea>();
    stateInspectBuoy_ = std::make_shared<states::StateInspectBuoy>();
    stateInspectPipes_ = std::make_shared<states::StateInspectPipes>();
    stateUpdateLocalization_ = std::make_shared<states::StateUpdateLocalization>();

    statesMap_.insert({ states::ID::init, stateInit_ });
    statesMap_.insert({ states::ID::homing, stateHoming_ });
    statesMap_.insert({ states::ID::moveToWp, stateLatLong_ });
    statesMap_.insert({ states::ID::searchForObject, stateSearchObject_ });
    statesMap_.insert({ states::ID::crossGate, stateCrossGate_ });
    statesMap_.insert({ states::ID::searchBuoyArea, stateSearchBuoyArea_ });
    statesMap_.insert({ states::ID::inspectBuoy, stateInspectBuoy_ });
    statesMap_.insert({ states::ID::inspectPipes, stateInspectPipes_ });
    statesMap_.insert({ states::ID::updateLocalization, stateUpdateLocalization_ });

    // ***** STATES ***** //
    // Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->ctrlData = ctrlData_;
        state.second->systemStatus_ = systemStatus_;

        if (state.first==states::ID::init)
            state.second->stateTimeout=systemStatus_->conf.initStateTimeout;
        
        else if (state.first==states::ID::homing)
            state.second->stateTimeout=systemStatus_->conf.homingStateTimeout;
        
        else if (state.first==states::ID::moveToWp)
            state.second->stateTimeout=systemStatus_->conf.moveToWpStateTimeout;
        
        else if (state.first==states::ID::searchForObject)
            state.second->stateTimeout=systemStatus_->conf.searchForObjectStateTimeout;
        
        else if (state.first==states::ID::crossGate)
            state.second->stateTimeout=systemStatus_->conf.crossGateStateTimeout;
        
        else if (state.first==states::ID::searchBuoyArea)
            state.second->stateTimeout=systemStatus_->conf.searchBuoyAreaStateTimeout;
        
        else if (state.first==states::ID::inspectBuoy)
            state.second->stateTimeout=systemStatus_->conf.inspectBuoyStateTimeout;
        
        else if (state.first==states::ID::inspectPipes)
            state.second->stateTimeout=systemStatus_->conf.inspectPipesStateTimeout;
        
        else if (state.first==states::ID::updateLocalization)
            state.second->stateTimeout=systemStatus_->conf.updateLocalizationStateTimeout;

        state.second->SetFSM(&rFsm_);
    }
    // ADD STATES
    for (auto& state : statesMap_) {
        rFsm_.AddState(state.first, state.second.get());
    }
    // ENABLE TRANSITIONS
    for (auto& currentState : statesMap_) {
        for (auto& nextState : statesMap_) {
            if (nextState.first != currentState.first)
                rFsm_.EnableTransition(currentState.first, nextState.first, true);
        }
    }
    // rFsm_.SetInitState(mission::states::ID::init);
}

void MissionController::SetTaskDataFSM()
{
    for (auto& state : statesMap_) {
        state.second->taskData_ = taskData_;
    }
}

void MissionController::ResetTaskDataFSM()
{
    taskData_ = nullptr;
    SetTaskDataFSM();
    // if (systemStatus_->conf.simCtrlStation) {
    //     simCtrlStationTimer_->reset();
    // }
    // rFsm_.SetInitState(mission::states::ID::init);
}
}
