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

    statesMap_.insert({ states::ID::init, stateInit_ });
    statesMap_.insert({ states::ID::homing, stateHoming_ });
    statesMap_.insert({ states::ID::moveToWp, stateLatLong_ });
    statesMap_.insert({ states::ID::searchForObject, stateSearchObject_ });
    statesMap_.insert({ states::ID::crossGate, stateCrossGate_ });
    statesMap_.insert({ states::ID::searchBuoyArea, stateSearchBuoyArea_ });
    statesMap_.insert({ states::ID::inspectBuoy, stateInspectBuoy_ });
    statesMap_.insert({ states::ID::inspectPipes, stateInspectPipes_ });

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

    int msRunPeriod = 1.0 / (controlLoopRate) * 1000;
    // std::cout << "Controller Rate: " << conf_->controlLoopRate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&MissionController::Run, this));

    if (systemStatus_->conf.simCtrlStation) {
        SimulateMissionCmdFromFile();
        SetTaskDataFSM();
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No control station");
    }
    if (systemStatus_->conf.simBridge)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No bridge");
    if (systemStatus_->conf.simKcl)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No KCL");
    if (systemStatus_->conf.simPerception)
        RCLCPP_WARN(this->get_logger(), "[DEBUG SETTING] --> No Perception");
};

void MissionController::StatusPub()
{
    // MISSION STATUS
    auv_core_helper::msg::MissionStatus status;
    status.stamp = this->now();
    status.task_benchmark = taskData_->taskType;
    if (rFsm_.GetCurrentStateName() == states::ID::init) {
        status.state = states::ID::init;
    } else {
        status.state = taskData_->taskPhases.front().first; // rFsm_.GetCurrentStateName();
        status.state_object = taskData_->taskPhases.front().second;
    }
    status.requests.obstacles = ctrlData_->perceptionData.enableDtcObstacles;
    status.requests.buoys = ctrlData_->perceptionData.enableDtcBuoys;

    missionStatusPub_->publish(status);
}

void MissionController::Run()
{
    //=== Check if the system is alive ===
    systemStatus_->UpdateStatus(this->get_clock()->now());
    if (!systemStatus_->IsAlive()) {
        RCLCPP_WARN(this->get_logger(), "System not alive. Perception: %d, KCL: %d, Bridge: %d",
            systemStatus_->perceptionAlive, systemStatus_->kclAlive, systemStatus_->bridgeAlive);
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
        RCLCPP_WARN(this->get_logger(), "Waiting for task data to be set by ctrl station");
        return;
    }

    //=== Send command to KCL ===
    if (ctrlData_->kclData.newCommand) {
        ctrlData_->kclData.newCommand = false;
        ctrlData_->kclData.executingCommand = true;

        auto options = rclcpp_action::Client<auv_core_helper::action::SetKCL>::SendGoalOptions();
        options.result_callback = [this](const rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::WrappedResult& result) {
            RCLCPP_INFO(this->get_logger(), "Command execution result: %d", static_cast<int>(result.code));
        };

        auv_core_helper::action::SetKCL::Goal cmd;
        cmd = ctrlData_->kclData.kcl_command;
        if (systemStatus_->conf.debugPrints) {
            RCLCPP_INFO(this->get_logger(), "Sending command to KCL [%s], %f, %f, %f", cmd.desired_state.c_str(),
                cmd.position.latitude, cmd.position.longitude, cmd.depth);
        }

        if (systemStatus_->conf.simKcl) {
            ctrlData_->inertialF_linearPosition.latitude = cmd.position.latitude;
            ctrlData_->inertialF_linearPosition.longitude = cmd.position.longitude;
            ctrlData_->depth = cmd.depth;
            return;
        }

        if (setKCLClient_->wait_for_action_server(std::chrono::seconds(1))) {
            setKCLClient_->async_send_goal(cmd, options);
        } else {
            RCLCPP_WARN(this->get_logger(), "KCL is not ready to rcv a cmd.");
            // FAIL?
        }
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
        RCLCPP_INFO(this->get_logger(), "Configuration loaded from file: %s", confPath.c_str());
        RCLCPP_INFO(this->get_logger(), "simKcl: %d", systemStatus_->conf.simKcl);
        RCLCPP_INFO(this->get_logger(), "simPerception: %d", systemStatus_->conf.simPerception);
        RCLCPP_INFO(this->get_logger(), "simBridge: %d", systemStatus_->conf.simBridge);
        RCLCPP_INFO(this->get_logger(), "simCtrlStation: %d", systemStatus_->conf.simCtrlStation);
        RCLCPP_INFO(this->get_logger(), "debugPrints: %d", systemStatus_->conf.debugPrints);

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
    std::cerr << "===== TaskBenchMark " << taskData_->taskType << " =====" << std::endl;
    std::cerr << *taskData_ << std::endl;
}
}
