#include "mission_ctrl/mission_controller.hpp"

namespace mission {
MissionController::MissionController(std::string conf_filename)
    : Node("mission_control_node")
{
    ctrlData_ = std::make_shared<ControlData>();

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
        this, "set_kcl");

    if (this->get_clock()->get_clock_type() == 1) {
        lastPerceptionTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    } else {
        lastPerceptionTime_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    }
    // lastPerceptionTime_ = this->get_clock()->now();
    perceptionSub_ = this->create_subscription<auv_core_helper::msg::DtcList>(
        auv_core_helper::topicnames::objects, rclcpp::SystemDefaultsQoS(),
        std::bind(&MissionController::PerceptionCB, this, std::placeholders::_1));

    missionCommandService_ = this->create_service<auv_core_helper::srv::MissionCommand>(
        "mission_command",
        std::bind(&MissionController::MissionCommandCB, this, std::placeholders::_1, std::placeholders::_2));

    SetUpFSM();

    double controlLoopRate = 1.0; // Hz
    int msRunPeriod = 1.0 / (controlLoopRate) * 1000;
    // std::cout << "Controller Rate: " << conf_->controlLoopRate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&MissionController::Run, this));

#ifdef NO_CTRL_STATION
    fileName_ = conf_filename;
    if (!LoadConfiguration()) {
        std::cerr << "Failed to load configuration from file" << std::endl;
        return;
    }
    // Print configuration
    std::cerr << "===== TaskBenchMark " << taskData_->taskType << " =====" << std::endl;
    std::cerr << *taskData_ << std::endl;
    UpdateFSM();
#endif
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

#ifndef DEBUG
    rclcpp::Time now = this->get_clock()->now();
    if (lastPerceptionTime_ > (now - rclcpp::Duration::from_seconds(5.0))) {
        ctrlData_->perceptionData.isAlive = true;
    } else {
        ctrlData_->perceptionData.isAlive = false;
    }
#else
    ctrlData_->perceptionData.isAlive = true;
#endif

    // Switch State (if something happens)
    rFsm_.SwitchState();
    // Process Events
    rFsm_.ProcessEventQueue();
    // Execute current state
    rFsm_.ExecuteState();

    if (taskData_ == nullptr) {
        return;
    }

    StatusPub();

    // KCL COMMAND
    if (ctrlData_->kclData.newCommand) {
        ctrlData_->kclData.newCommand = false;
        ctrlData_->kclData.executingCommand = true;

#ifndef DEBUG
        if (!setKCLClient_->wait_for_action_server(std::chrono::seconds(3))) {
            // HUGE FAIL
            RCLCPP_WARN(this->get_logger(), "Action server not available.");
            return;
        }
#endif

        auto options = rclcpp_action::Client<auv_core_helper::action::SetKCL>::SendGoalOptions();
        options.result_callback = [this](const rclcpp_action::ClientGoalHandle<auv_core_helper::action::SetKCL>::WrappedResult& result) {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                // HUGE FAIL
                RCLCPP_ERROR(this->get_logger(), "Command execution failed.");
            }
            ctrlData_->kclData.executingCommand = false;
        };

        auv_core_helper::action::SetKCL::Goal cmd;
        cmd.desired_state = "WAYPOINT_NAVIGATION";
        cmd.data = ctrlData_->kclData.kcl_command;
        std::cout << "Sending command: ros2 action send_goal /set_kcl_state auv_core_helper/action/SetKCL \"{desired_state: '"
                  << cmd.desired_state << "', data: {latitude: " << cmd.data.latitude << ", longitude: " << cmd.data.longitude << "}}\"" << std::endl;
        setKCLClient_->async_send_goal(cmd, options);
    }
};

void MissionController::SetUpFSM()
{
    // ***** STATES ***** //
    // Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->ctrlData = ctrlData_;
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

void MissionController::UpdateFSM()
{
    for (auto& state : statesMap_) {
        state.second->taskData_ = taskData_;
    }
}

void MissionController::PerceptionCB(const auv_core_helper::msg::DtcList::SharedPtr msg)
{
    lastPerceptionTime_ = this->get_clock()->now();
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

    //     LatLong position
    // string color
    // float64 color_confidence

    // GenericObstacle[] obstacles
    // Buoy[]  buoys
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
    UpdateFSM();
}

bool MissionController::LoadConfiguration()
{
    libconfig::Config confObj;
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("mission_ctrl");
    std::string confPath = package_share_directory + "/conf/" + fileName_;

    try {
        confObj.readFile(confPath.c_str());
        uint tbm_id;
        if (!ctb::GetParam(confObj, tbm_id, "tbm"))
            return false;
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
            return false;
        }
        return taskData_->ConfigureFromFile(confObj);
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        std::cerr << "  Path: '" << confPath << "'. Make sure the file exists and is readable." << std::endl;
        return false;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return false;
    }
}
}
