#include "mission_ctrl/mission_controller.hpp"

namespace mission {
MissionController::MissionController(std::string conf_filename)
    : Node("mission_control_node")
{
    ctrlData_ = std::make_shared<ControlData>();

    fileName_ = conf_filename;
    if (!LoadConfiguration(taskData_)) {
        std::cerr << "Failed to load configuration from file" << std::endl;
        return;
    }

    // Print configuration
    std::cerr << "===== TaskBenchMark " << taskData_->taskType << " =====" << std::endl;
    std::cerr << *taskData_ << std::endl;

    stateInit_ = std::make_shared<states::StateInit>();
    stateHoming_ = std::make_shared<states::StateHoming>();
    stateLatLong_ = std::make_shared<states::StateLatLong>();
    stateSearchObject_ = std::make_shared<states::StateSearchObject>();
    stateCrossGate_ = std::make_shared<states::StateCrossGate>();

    statesMap_.insert({ states::ID::init, stateInit_ });
    statesMap_.insert({ states::ID::homing, stateHoming_ });
    statesMap_.insert({ states::ID::moveToWp, stateLatLong_ });
    statesMap_.insert({ states::ID::searchForObject, stateSearchObject_ });
    statesMap_.insert({ states::ID::crossGate, stateCrossGate_ });


    obstaclesSub_ = this->create_subscription<auv_core_helper::msg::ObstacleList>(
        auv_core_helper::topicnames::obstacles, rclcpp::SystemDefaultsQoS(),
        [this](auv_core_helper::msg::ObstacleList::SharedPtr msg) {
            obstacles_ = *msg;
            RCLCPP_INFO(this->get_logger(), "Received obstacle list with %zu obstacles.", obstacles_.data.size());
        });
    auv_core_helper::msg::ObstacleList obstacles_;
    rclcpp::Subscription<auv_core_helper::msg::ObstacleList>::SharedPtr obtaclesSub_;

    SetUpFSM();

    double controlLoopRate = 0.5;
    int msRunPeriod = 1.0 / (controlLoopRate) * 1000;
    // std::cout << "Controller Rate: " << conf_->controlLoopRate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&MissionController::Run, this));
};

void MissionController::Run()
{   
    // Switch State (if something happens)
    rFsm_.SwitchState();
    // Process Events
    rFsm_.ProcessEventQueue();
    // Execute current state
    rFsm_.ExecuteState();
};

void MissionController::SetUpFSM()
{
    // ***** STATES ***** //
    // Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->ctrlData = ctrlData_;
        state.second->taskData_ = taskData_;
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

bool MissionController::LoadConfiguration(std::shared_ptr<TaskBenchmarkSettings>& conf)
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
            conf = std::make_shared<Inspection>();
            break;
        case 2:
            conf = std::make_shared<Intervention>();
            break;
        case 3:
            conf = std::make_shared<InspectionAndIntervention>();
            break;
        default:
            std::cerr << "Invalid tbm id: " << tbm_id << std::endl;
            return false;
        }
        return conf->ConfigureFromFile(confObj);
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
