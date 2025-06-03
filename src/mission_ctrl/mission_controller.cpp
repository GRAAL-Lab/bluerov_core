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

    SetUpFSM();
    systemStatus_->lastStateSwitchTime = this->get_clock()->now();

    int msRunPeriod = 1.0 / (systemStatus_->conf.ctrlRate) * 1000;
    // std::cout << "Controller Rate: " << conf_->controlLoopRate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&MissionController::Run, this));

    if (systemStatus_->conf.simBridge)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No bridge");
    if (systemStatus_->conf.simKcl)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No KCL");
    if (systemStatus_->conf.simPerception)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No Perception");

    if (systemStatus_->conf.simCtrlStation) {
        SimulateMissionCmdFromFile();
        SetTaskDataFSM();
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No control station");
    } else {
        RCLCPP_WARN(this->get_logger(), "Waiting for task data to be set by ctrl station");
    }
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
    //=== Check if the system is alive ===
    if (!systemStatus_->IsAlive(this->get_clock()->now())) {
        if (systemStatus_->kclAlive && !systemStatus_->kclActionServerAlive && !systemStatus_->conf.simKcl) {
            if (setKCLClient_->wait_for_action_server(std::chrono::seconds(1))) {
                // ok, back online
                systemStatus_->kclActionServerAlive = true;
            } else {
                RCLCPP_WARN(this->get_logger(), "KCL action server is not alive, waiting for it to be ready.");
                return;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "System not alive. Perception: %d, KCL: %d, Bridge: %d",
                systemStatus_->perceptionAlive, systemStatus_->kclAlive, systemStatus_->bridgeAlive);
            return;
        }
    }
    if (systemStatus_->WasDeadForTooLong(this->get_clock()->now())) {
        RCLCPP_ERROR(this->get_logger(), "System was dead for too long, sending only surface cmd.");
        auv_core_helper::action::SetKCL::Goal cmd;
        cmd.desired_state = "WAYPOINT_NAVIGATION";
        cmd.position.latitude = ctrlData_->inertialF_linearPosition.latitude;
        cmd.position.longitude = ctrlData_->inertialF_linearPosition.longitude;
        cmd.depth = taskData_->surfaceDepth;

        if (setKCLClient_->wait_for_action_server(std::chrono::seconds(1))) {
            auto options = rclcpp_action::Client<auv_core_helper::action::SetKCL>::SendGoalOptions();
            setKCLClient_->async_send_goal(cmd, options);
        }
        return;
    }
    //=== Check if the system is stuck in a state ===
    auto now = this->get_clock()->now();
    if (rFsm_.GetCurrentStateName() != rFsm_.GetNextStateName()) {
        systemStatus_->lastStateSwitchTime = now;
    } else {
        double timeSinceLastSwitch = now.seconds() - systemStatus_->lastStateSwitchTime.seconds();
        if (timeSinceLastSwitch > 20.0 && std::fmod(timeSinceLastSwitch, 10.0) < 1.0) {
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
    if (ctrlData_->kclData.cancelCommand) {
        RCLCPP_WARN(this->get_logger(), "Received cancel cmd but not implemented yet.");
        ctrlData_->kclData.cancelCommand = false;
    }
    if (ctrlData_->kclData.newCommand) {
        auv_core_helper::action::SetKCL::Goal cmd;
        cmd = ctrlData_->kclData.new_kcl_command;

        ctrlData_->kclData.newCommand = false;
        ctrlData_->kclData.new_kcl_command = auv_core_helper::action::SetKCL::Goal();
        ctrlData_->kclData.executingCommand = true;
        ctrlData_->kclData.last_kcl_command = cmd;

        auto options = rclcpp_action::Client<auv_core_helper::action::SetKCL>::SendGoalOptions();
        options.result_callback = [this](const rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::WrappedResult& result) {
            RCLCPP_INFO(this->get_logger(), "Command execution result: %d", static_cast<int>(result.code));
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED && ctrlData_->kclData.last_kcl_command.desired_state == "WAYPOINT_NAVIGATION") {
                double distance, azimuthRad;
                ctb::DistanceAndAzimuthRad(ctrlData_->inertialF_linearPosition,
                    { ctrlData_->kclData.last_kcl_command.position.latitude, ctrlData_->kclData.last_kcl_command.position.longitude },
                    distance, azimuthRad);
                if (distance > systemStatus_->conf.latlongTolerance) {
                    RCLCPP_WARN(this->get_logger(),
                        "YOUSSEF WTF, kcl says it completed the command but the distance is [%f], which is greater than the tolerance in the conf file[%f]",
                        distance, systemStatus_->conf.latlongTolerance);
                }

            } else {
                RCLCPP_ERROR(this->get_logger(), "NOT HANDLED YEY: Command execution failed with code: %d", static_cast<int>(result.code));
            }
        };

        if (systemStatus_->conf.debugPrints) {
            RCLCPP_INFO(this->get_logger(), "Sending command to KCL [%s], %f, %f, %f", cmd.desired_state.c_str(),
                cmd.position.latitude, cmd.position.longitude, cmd.depth);
        }

        if (systemStatus_->conf.simKcl) {
            RCLCPP_WARN(this->get_logger(), "Received cmd for KCL but simulating it, so setting it as completed.");
            ctrlData_->inertialF_linearPosition.latitude = cmd.position.latitude;
            ctrlData_->inertialF_linearPosition.longitude = cmd.position.longitude;
            ctrlData_->depth = cmd.depth;
            return;
        }

        auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(systemStatus_->conf.kclLivenessTimeout));
        if (setKCLClient_->wait_for_action_server(timeout)) {
            setKCLClient_->async_send_goal(cmd, options);
        } else {
            RCLCPP_WARN(this->get_logger(), "KCL is not ready to rcv a cmd.");
            // FAIL?
        }
    }

    if (taskData_->taskPhases.empty()) {
        taskData_ = nullptr;
        SetTaskDataFSM();
    }
};

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

void MissionController::PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg)
{
    systemStatus_->lastPerceptionTime = this->get_clock()->now();
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
    }
}

void MissionController::KclCB(const auv_core_helper::msg::KclStatus::SharedPtr msg)
{
    (void)msg;
    systemStatus_->lastKCLTime = this->get_clock()->now();
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
    SetTaskDataFSM();
    response->res = true;
    rFsm_.SetInitState(mission::states::ID::init);
    systemStatus_->systemAlive = true;
    stateHoming_->homePosition.latitude = ctrlData_->inertialF_linearPosition.latitude;
    stateHoming_->homePosition.longitude = ctrlData_->inertialF_linearPosition.longitude;
    RCLCPP_INFO(this->get_logger(), "Mission command received, tbm_id: %d", request->tbm_id);
    RCLCPP_INFO(this->get_logger(), "Set home position to: %f, %f",
        stateHoming_->homePosition.latitude, stateHoming_->homePosition.longitude);

    if (systemStatus_->conf.debugPrints) {
        std::cerr << "===== TaskBenchMark " << taskData_->taskType << " =====" << std::endl;
        std::cerr << *taskData_ << std::endl;
    }
}

void MissionController::PoseCB(const auv_core_helper::msg::PoseStamped::SharedPtr msg)
{
    systemStatus_->lastBridgeTime = this->get_clock()->now();
    ctrlData_->inertialF_linearPosition.latitude = msg->position.latitude;
    ctrlData_->inertialF_linearPosition.longitude = msg->position.longitude;
    ctrlData_->depth = msg->depth;
    ctrlData_->bodyF_angularPosition.RPY(msg->roll,
        msg->pitch, msg->yaw);
}

void MissionController::LoadConfiguration()
{
    // Load configuration from file
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("mission_ctrl");
    std::string confPath = package_share_directory + "/conf/" + "system.conf";
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

    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        std::cerr << "  Path: '" << confPath << "'. Make sure the file exists and is readable." << std::endl;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
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
}
