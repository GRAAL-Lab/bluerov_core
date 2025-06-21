#include "mission_ctrl/mission_controller.hpp"

namespace mission {
MissionController::MissionController()
    : Node("mission_control_node")
{
    ctrlData_ = std::make_shared<ControlData>();
    systemStatus_ = std::make_shared<SystemStatus>(this->get_clock()->get_clock_type());
    LoadConfiguration(); // REQUIRES SYSTEM STATUS TO BE INITIALIZED

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

    kclSendGoalOptions_ = rclcpp_action::Client<auv_core_helper::action::SetKCL>::SendGoalOptions();
    kclSendGoalOptions_.result_callback = std::bind(&MissionController::ActionResultCallback, this, std::placeholders::_1);
    kclSendGoalOptions_.feedback_callback = std::bind(&MissionController::ActionFeedbackCallback, this,
        std::placeholders::_1, std::placeholders::_2);

    SetUpFSM();
    systemStatus_->lastSystemTime = this->get_clock()->now();
    systemStatus_->lastStateSwitchTime = this->get_clock()->now();
    systemStatus_->timeOutsideSafetyArea = this->get_clock()->now();

    int msRunPeriod = 1.0 / (systemStatus_->conf.ctrlRate) * 1000;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&MissionController::Run, this));

    if (systemStatus_->conf.simCtrlStation) {
        SimulateMissionCmdFromFile();
        for (const auto& point : taskData_->buoysArea.points) {
            if (!IsPointWithinBoundaries(point)) {
                RCLCPP_ERROR(this->get_logger(), "Buoys area point is outside of safety boundaries: [%f, %f]",
                    point.latitude, point.longitude);
                // kill the node
                rclcpp::shutdown();
                return;
            }
        }
        SetTaskDataFSM();
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No control station");
    } else {
        RCLCPP_WARN(this->get_logger(), "Waiting for task data to be set by ctrl station");
    }

    if (systemStatus_->conf.simBridge)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No bridge");
    if (systemStatus_->conf.simKcl)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No KCL");
    if (systemStatus_->conf.simPerception)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No Perception");

    RCLCPP_INFO(this->get_logger(), "Waiting for components to be alive...");
    // while(!systemStatus_->IsSystemAlive(this->get_clock()->now()));
    // RCLCPP_INFO(this->get_logger(), "All components are alive, starting mission control node.");
};

void MissionController::StatusPub()
{
    auv_core_helper::msg::MissionStatus status;
    status.stamp = this->get_clock()->now();
    if (taskData_ == nullptr || taskData_->taskPhases.empty() || rFsm_.GetCurrentStateName() == states::ID::init) {
        status.state = states::ID::init;
        status.task_benchmark = "WAITING FOR CMD";
    } else {
        status.task_benchmark = taskData_->taskType;
        status.state = taskData_->taskPhases.front().first;
        status.state_object = taskData_->taskPhases.front().second;
    }
    status.requests.obstacles = ctrlData_->perceptionData.enableDtcObstacles;
    status.requests.buoys = ctrlData_->perceptionData.enableDtcBuoys;

    missionStatusPub_->publish(status);
}

void MissionController::Run()
{
    if (!systemStatus_->missionCtrlRunning) {
        if (systemStatus_->IsSystemAlive(this->get_clock()->now())) {
            systemStatus_->missionCtrlRunning = true;
            RCLCPP_INFO(this->get_logger(), "All components are alive, starting mission control node.");
        } else {
            return;
        }
    }
    // if (systemStatus_->conf.simBridge && !IsPointWithinBoundaries(ctrlData_->inertialF_linearPosition)) {
    //     RCLCPP_ERROR(this->get_logger(), "Vehicle is outside of safety boundaries: [%f, %f]",
    //         ctrlData_->inertialF_linearPosition.latitude, ctrlData_->inertialF_linearPosition.longitude);
    //     kclCmd("IDLE");
    // }

    //=== Check if the components are alive ===
    if (!systemStatus_->IsSystemAlive(this->get_clock()->now())) {
        RCLCPP_ERROR(this->get_logger(), "Components status: Bridge: %s, KCL: %s, Perception: %s, time since last system status: %.2f seconds",
            systemStatus_->bridgeAlive ? "Alive" : "Not Alive",
            systemStatus_->kclAlive ? "Alive" : "Not Alive",
            systemStatus_->perceptionAlive ? "Alive" : "Not Alive",
            (this->get_clock()->now() - systemStatus_->lastSystemTime).seconds());
        kclCmd("IDLE");
        return;
    }

    //=== Check if the system is stuck in a state ===
    auto now = this->get_clock()->now();
    if (rFsm_.GetCurrentStateName() != rFsm_.GetNextStateName()) {
        systemStatus_->lastStateSwitchTime = now;
        RCLCPP_INFO(this->get_logger(), "FSM switched to state %s", rFsm_.GetNextStateName().c_str());

        if (rFsm_.GetNextStateName() == states::ID::init) {
            if (systemStatus_->conf.simCtrlStation) {
                SimulateMissionCmdFromFile();
                SetTaskDataFSM();
            }
        }

    } else {
        double timeSinceLastSwitch = now.seconds() - systemStatus_->lastStateSwitchTime.seconds();
        // Get current state timeout value
        double currentStateTimeout = statesMap_[rFsm_.GetCurrentStateName()]->stateTimeout;
        if (timeSinceLastSwitch > currentStateTimeout && std::fmod(timeSinceLastSwitch, 10.0) < 1.0) {
            RCLCPP_WARN(this->get_logger(), "FSM in state %s for %i seconds", rFsm_.GetCurrentStateName().c_str(), (int)timeSinceLastSwitch);
        }
    }

    //=== Progress execution ===
    // Switch State (if something happens)
    rFsm_.SwitchState();
    // Process Events
    rFsm_.ProcessEventQueue();
    // Execute current state
    rFsm_.ExecuteState();

    // Publish status
    StatusPub();

    if (taskData_ == nullptr) {
        // RCLCPP_WARN(this->get_logger(), "Waiting for task data to be set by ctrl station");
        return;
    }

    //=== Send command to KCL ===
    if (ctrlData_->kclData.cancelCommand && !kclCmd("CANCEL")) {
        RCLCPP_WARN(this->get_logger(), " Tried to send cancel cmd but server is not available.");
        return;
    }
    if (!ctrlData_->kclData.currentCmd.sentToKcl && ctrlData_->kclData.currentCmd.goal.desired_state != "") {
        kclCmd();
    }

    if (taskData_->taskPhases.empty()) {
        taskData_ = nullptr;
        SetTaskDataFSM();
    }
};

void MissionController::SystemStatusCB(const auv_core_helper::msg::SystemStatus::SharedPtr msg)
{
    systemStatus_->bridgeAlive = systemStatus_->conf.simBridge || msg->bridge;
    systemStatus_->kclAlive = systemStatus_->conf.simKcl || msg->kcl;
    systemStatus_->perceptionAlive = systemStatus_->conf.simPerception || msg->perception;
    systemStatus_->lastSystemTime = this->get_clock()->now();
}

void MissionController::PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg)
{

    // Update perception data
    // for(auto& obstacle : msg->obstacles) {

    // }
    for (auto& buoy : msg->buoys) {
        Buoy b;
        b.detectionId = buoy.id;
        b.position.latitude = buoy.position.latitude;
        b.position.longitude = buoy.position.longitude;
        b.radius = buoy.radius;
        b.color = buoy.color;
        b.colorConfidence = buoy.color_confidence;

        ctrlData_->perceptionData.detectedBuoys[buoy.id] = b;
        std::cerr << "Detected buoy: " << buoy.id << " at position: ["
                  << buoy.position.latitude << ", " << buoy.position.longitude << "]\n";
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

    for (const auto& point : taskData_->buoysArea.points) {
        if (!IsPointWithinBoundaries(point)) {
            response->res = false;
            response->text = "Buoys area point is outside of safety boundaries: [" + std::to_string(point.latitude) + ", " + std::to_string(point.longitude) + "]";
            RCLCPP_ERROR(this->get_logger(), "%s", response->text.c_str());
            return;
        }
    }

    if (ctrlData_->depth < 1.0) {
        taskData_->surfaceDepth = ctrlData_->depth;
        RCLCPP_INFO(this->get_logger(), "Setting surface depth: current depth: [%f]", taskData_->surfaceDepth);
    } else {
        taskData_->surfaceDepth = 0.5; // Default surface depth if not set
        RCLCPP_WARN(this->get_logger(), "Setting surface depth: current depth is too deep [%f], setting to: [%f]", ctrlData_->depth, taskData_->surfaceDepth);
    }

    SetTaskDataFSM();
    response->res = true;
    rFsm_.SetInitState(mission::states::ID::init);
    stateHoming_->homePosition.latitude = ctrlData_->inertialF_linearPosition.latitude;
    stateHoming_->homePosition.longitude = ctrlData_->inertialF_linearPosition.longitude;
    RCLCPP_INFO(this->get_logger(), "Mission command received, tbm_id: %d", request->tbm_id);
    RCLCPP_INFO(this->get_logger(), "Set home position as current one: [%f, %f]",
        stateHoming_->homePosition.latitude, stateHoming_->homePosition.longitude);

    RCLCPP_INFO(this->get_logger(), " ===== TaskBenchMark %s =====", taskData_->taskType.c_str());
    RCLCPP_INFO(this->get_logger(), "%s", *taskData_);
}

void MissionController::PoseCB(const auv_core_helper::msg::PoseStamped::SharedPtr msg)
{
    ctrlData_->inertialF_linearPosition.latitude = msg->position.latitude;
    ctrlData_->inertialF_linearPosition.longitude = msg->position.longitude;
    ctrlData_->depth = msg->depth;
    ctrlData_->bodyF_angularPosition.RPY(msg->roll,
        msg->pitch, msg->yaw);

    if (systemStatus_->conf.simCtrlStation && !stateHoming_->homePositionSet) {
        stateHoming_->homePositionSet = true;
        if (ctrlData_->depth < 1.0) {
            taskData_->surfaceDepth = ctrlData_->depth;
            RCLCPP_INFO(this->get_logger(), "Setting surface depth: current depth: [%f]", taskData_->surfaceDepth);
        } else {
            taskData_->surfaceDepth = 0.5; // Default surface depth if not set
            RCLCPP_WARN(this->get_logger(), "Setting surface depth: current depth is too deep, setting to: [%f]", taskData_->surfaceDepth);
        }
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> Simulating control station, setting home position as the one in front of the dock.");
        stateHoming_->homePosition.latitude = 44.09595617190768;
        stateHoming_->homePosition.longitude = 9.864626568817506;
        RCLCPP_INFO(this->get_logger(), "Set home position as current one: [%f, %f]",
            stateHoming_->homePosition.latitude, stateHoming_->homePosition.longitude);
    }

    // Check if the vehicle has finally reached the safety area
    if (!systemStatus_->vehicleReachedSafetyArea && IsPointWithinBoundaries(ctrlData_->inertialF_linearPosition)) {
        systemStatus_->vehicleReachedSafetyArea = true;
        RCLCPP_INFO(this->get_logger(), "Vehicle reached safety area!");
    }

    // Check if the vehicle has been outside the safety area for too long
    if (!systemStatus_->vehicleReachedSafetyArea && !IsPointWithinBoundaries(ctrlData_->inertialF_linearPosition)) {
        auto now = this->get_clock()->now();
        if ((now - systemStatus_->timeOutsideSafetyArea).seconds() > 15) {
            RCLCPP_ERROR(this->get_logger(), "Vehicle has been outside the safety area for too long.");
            // TODO HANDLE THIS
        }
    }

    // Check if the vehicle exited the safety area
    if (systemStatus_->vehicleReachedSafetyArea && !IsPointWithinBoundaries(ctrlData_->inertialF_linearPosition)) {
        RCLCPP_ERROR(this->get_logger(), "Vehicle is outside of safety boundaries: [%f, %f]",
            ctrlData_->inertialF_linearPosition.latitude, ctrlData_->inertialF_linearPosition.longitude);
        ctrlData_->kclData.currentCmd = mission::kclCmd();
        ctrlData_->kclData.currentCmd.goal.desired_state = "IDLE";

        rFsm_.SetInitState(mission::states::ID::init);
    }
}

bool MissionController::IsPointWithinBoundaries(const ctb::LatLong& point)
{
    if (systemStatus_->conf.safetyBoundary.size() < 3) {
        throw std::runtime_error("Safety boundary must have at least 3 points.");
    }

    std::vector<Eigen::Vector3d> polygon;
    for (auto& latlong : systemStatus_->conf.safetyBoundary) {
        Eigen::Vector3d bodyF_point;
        ctb::LatLong2LocalNED(latlong, 0, point, bodyF_point);
        polygon.push_back(bodyF_point);
    }

    bool pos = false, neg = false;
    int i = 0;
    for (auto point : polygon) {
        auto nextPoint = polygon[(i + 1) % polygon.size()];
        double cross = (nextPoint.x() - point.x()) * (-point.y()) - (nextPoint.y() - point.y()) * (-point.x());
        if (cross < 0)
            neg = true;
        if (cross > 0)
            pos = true;
        if (pos && neg)
            return false; // point is outside
        i++;
    }

    return true;
}

bool MissionController::kclCmd(std::string cmd)
{
    if (cmd == "IDLE") {
        RCLCPP_WARN(this->get_logger(), "Sending IDLE cmd and moving to INIT state.");
        systemStatus_->missionCtrlRunning = false;
        rFsm_.SetInitState(mission::states::ID::init);
        auv_core_helper::action::SetKCL::Goal cmd;
        cmd.desired_state = "IDLE";
        while (!setKCLClient_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for KCL action server to be ready...");
        }
        setKCLClient_->async_send_goal(cmd, kclSendGoalOptions_);
    } else if (cmd == "CANCEL") {
        auv_core_helper::action::SetKCL::Goal cmd;
        cmd.desired_state = "CANCEL";
        if (!setKCLClient_->wait_for_action_server(std::chrono::seconds(1))) {
            // systemStatus_->kclActionServerAlive = false;
            return false;
        }
        setKCLClient_->async_send_goal(cmd, kclSendGoalOptions_);
        ctrlData_->kclData.cancelCommand = false;
        return true;
    } else {

        auv_core_helper::action::SetKCL::Goal cmd;
        cmd = ctrlData_->kclData.currentCmd.goal;

        if (systemStatus_->conf.debugPrints) {
            if (cmd.desired_state == "WAYPOINT_NAVIGATION") {
                RCLCPP_INFO(this->get_logger(), "Sending command to KCL [%s], %f, %f, %f", cmd.desired_state.c_str(),
                    cmd.position.latitude, cmd.position.longitude, cmd.depth);
            } else {
                RCLCPP_INFO(this->get_logger(), "Sending command to KCL [%s], %s", cmd.desired_state.c_str());
            }
        }

        if (systemStatus_->conf.simKcl) {
            RCLCPP_WARN(this->get_logger(), "Received cmd for KCL but simulating it, so setting it as completed.");
            if (cmd.desired_state == "WAYPOINT_NAVIGATION") {
                ctrlData_->inertialF_linearPosition.latitude = cmd.position.latitude;
                ctrlData_->inertialF_linearPosition.longitude = cmd.position.longitude;
                ctrlData_->depth = cmd.depth;
            }
            return true;
        }

        auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(systemStatus_->conf.kclLivenessTimeout));
        if (setKCLClient_->wait_for_action_server(timeout)) {
            setKCLClient_->async_send_goal(cmd, kclSendGoalOptions_);
        } else {
            // systemStatus_->kclActionServerAlive = false;
            RCLCPP_WARN(this->get_logger(), "KCL is not ready to rcv a cmd.");
            return false;
        }

        ctrlData_->kclData.currentCmd.sentToKcl = true;
    }
}

void MissionController::ActionResultCallback(const rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::WrappedResult& result)
{
    ctrlData_->kclData.currentCmd.result.success = result.result->success;
    ctrlData_->kclData.currentCmd.result.message = result.result->message;
    RCLCPP_INFO(this->get_logger(), "KCL command RESULT [%s] with message [%s]",
        ctrlData_->kclData.currentCmd.result.success ? "SUCCESS" : "FAILURE",
        ctrlData_->kclData.currentCmd.result.message.c_str());
    if (result.result->success) {
        ctrlData_->kclData.currentCmd.completed = false;
    }
}

void MissionController::ActionFeedbackCallback(
    rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::SharedPtr,
    const std::shared_ptr<const auv_core_helper::action::SetKCL::Feedback>& feedback)
{
    ctrlData_->kclData.currentCmd.feedback.actual_state = feedback->actual_state;
    ctrlData_->kclData.currentCmd.feedback.action_progress = feedback->action_progress;

    if (systemStatus_->conf.debugPrints) {
        RCLCPP_INFO(this->get_logger(), "KCL command FEEDBACK: [%s] with progress %.2f",
            ctrlData_->kclData.currentCmd.feedback.actual_state.c_str(),
            ctrlData_->kclData.currentCmd.feedback.action_progress);
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

        ctb::GetParam(confObj, systemStatus_->conf.depthTolerance, "depth_tolerance");
        ctb::GetParam(confObj, systemStatus_->conf.latlongTolerance, "latlong_tolerance");
        ctb::GetParam(confObj, systemStatus_->conf.ctrlRate, "ctrl_rate");
        ctb::GetParam(confObj, systemStatus_->conf.bridgeLivenessTimeout, "bridge_liveness_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.perceptionLivenessTimeout, "perception_liveness_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.kclLivenessTimeout, "kcl_liveness_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.systemLivenessTimeout, "system_liveness_timeout");
        ctb::GetParam(confObj, systemStatus_->conf.debugBuoys, "buoysDebug");

        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& safetyBoundarySetting = root["safetyBoundary"];
        for (int i = 0; i < safetyBoundarySetting.getLength(); ++i) {
            const libconfig::Setting& point = safetyBoundarySetting[i];
            Eigen::VectorXd latLongTmp;
            ctb::GetParamVector(point, latLongTmp, "point");
            systemStatus_->conf.safetyBoundary.push_back(ctb::LatLong(latLongTmp[0], latLongTmp[1]));
        }

        if (systemStatus_->conf.simBridge) {
            Eigen::VectorXd latLongTmp;
            ctb::GetParamVector(confObj, latLongTmp, "startingPositionDebug");
            ctrlData_->inertialF_linearPosition.latitude = latLongTmp[0];
            ctrlData_->inertialF_linearPosition.longitude = latLongTmp[1];
        }

        if (systemStatus_->conf.debugBuoys) {
            const libconfig::Setting& buoysSetting = root["buoysStonefishPositions"];
            for (int i = 0; i < buoysSetting.getLength(); ++i) {
                const libconfig::Setting& point = buoysSetting[i];
                Eigen::VectorXd localTmp;
                ctb::GetParamVector(point, localTmp, "point");
                ctb::LatLong stonefishCentroid = ctb::LatLong(44.095952330602564, 9.865115308770484); // from update pose stonefish in stonefish utils
                ctb::LatLong latLongTmp;
                double alt;
                ctb::LocalNED2LatLong(localTmp, stonefishCentroid, latLongTmp, alt);
                systemStatus_->conf.debugBuoysPositions.push_back(latLongTmp);
            }
            std::cerr << "Debug buoys true positions:" << std::endl;
            for (const auto& point : systemStatus_->conf.debugBuoysPositions) {
                std::cerr << "  - [" << point.latitude << ", " << point.longitude << "]" << std::endl;
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
        RCLCPP_INFO(this->get_logger(), "bridgeLivenessTimeout: %f", systemStatus_->conf.bridgeLivenessTimeout);
        RCLCPP_INFO(this->get_logger(), "perceptionLivenessTimeout: %f", systemStatus_->conf.perceptionLivenessTimeout);
        RCLCPP_INFO(this->get_logger(), "kclLivenessTimeout: %f", systemStatus_->conf.kclLivenessTimeout);
        RCLCPP_INFO(this->get_logger(), "systemLivenessTimeout: %f", systemStatus_->conf.systemLivenessTimeout);
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

    // Print configuration
    if (systemStatus_->conf.debugPrints) {
        std::cerr << "===== TaskBenchMark " << taskData_->taskType << " =====" << std::endl;
        std::cerr << *taskData_ << std::endl;
    }
}

void MissionController::SetUpFSM()
{
    // ***** STATES ***** //
    // Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->ctrlData = ctrlData_;
        state.second->systemStatus_ = systemStatus_;
        // state.second->taskData_ = taskData_;
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
    rFsm_.SetInitState(mission::states::ID::init);
}

void MissionController::SetTaskDataFSM()
{
    for (auto& state : statesMap_) {
        state.second->taskData_ = taskData_;
    }
}
}
