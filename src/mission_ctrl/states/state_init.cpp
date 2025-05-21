#include "mission_ctrl/states/state_init.hpp"

namespace mission {

namespace states {

    StateInit::StateInit()
    {
    }

    StateInit::~StateInit() { }

    fsm::retval StateInit::OnEntry()
    {
        doneInit = false;
        std::cerr << "Waiting for task data to be set" << std::endl;
        return fsm::ok;
    }

    fsm::retval StateInit::Execute()
    {
        std::cerr << "executing init state" << std::endl;
        if (taskData_ == nullptr) {

            return fsm::ok;
        }

        if (!doneInit) {
            while (!taskData_->taskPhases.empty())
                taskData_->taskPhases.pop();

            if (taskData_->taskType == taskBenchmarks::INSPECTION) {
                taskData_->taskPhases.push(std::make_pair(states::ID::init, ""));
                taskData_->taskPhases.push(std::make_pair(states::ID::moveToWp, opis::uavWaypoint));
                taskData_->taskPhases.push(std::make_pair(states::ID::searchForObject, opis::gate));
                taskData_->taskPhases.push(std::make_pair(states::ID::crossGate, ""));
                // taskData_->taskPhases.push_back(std::make_pair(states::ID::searchBuoyArea, ""));
                // taskData_->taskPhases.push_back(std::make_pair(states::ID::moveToWp, opis::pipelineStructure));
                // taskData_->taskPhases.push_back(std::make_pair(states::ID::inspectPipes, ""));
                taskData_->taskPhases.push(std::make_pair(states::ID::homing, ""));
            } else if (taskData_->taskType == taskBenchmarks::INTERVENTION) {
                taskData_->taskPhases.push(std::make_pair(states::ID::init, ""));

            } else if (taskData_->taskType == taskBenchmarks::INSPECTION_AND_INTERVENTION) {
                taskData_->taskPhases.push(std::make_pair(states::ID::init, ""));
            }

            doneInit = true;
        }

#ifdef DEBUG
        return this->SetNextMissionState();
#endif

        if (systemStatus_->IsAlive()) {
            return this->SetNextMissionState();
        } else {
            return fsm::ok;
        }
    }

    fsm::retval StateInit::OnExit()
    {
        return fsm::ok;
    }
}
}
