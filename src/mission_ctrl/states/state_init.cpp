#include "mission_ctrl/states/state_init.hpp"


namespace mission {

namespace states {

    StateInit::StateInit()
    {
    }

    StateInit::~StateInit() { }

    fsm::retval StateInit::OnEntry()
    {   

        while(!taskData_->taskPhases.empty()) taskData_->taskPhases.pop();
        if(taskData_->taskType == taskBenchmarks::INSPECTION) {
            taskData_->taskPhases.push(std::make_pair(states::ID::moveToWp, opis::uavWaypoint));
            taskData_->taskPhases.push(std::make_pair(states::ID::searchForObject, opis::gate));
            taskData_->taskPhases.push(std::make_pair(states::ID::crossGate, ""));
            // taskData_->taskPhases.push_back(std::make_pair(states::ID::searchBuoyArea, ""));
            // taskData_->taskPhases.push_back(std::make_pair(states::ID::moveToWp, opis::pipelineStructure));
            // taskData_->taskPhases.push_back(std::make_pair(states::ID::inspectPipes, ""));
            taskData_->taskPhases.push(std::make_pair(states::ID::homing, ""));
        }

        // std::cerr << "\n\nInit with tasks: \n";
        // for (const auto& task : taskData_->taskPhases) {
        //     std::cerr << task.first << " - " << task.second << std::endl;
        // }
        return fsm::ok;        
    }

    fsm::retval StateInit::Execute()
    {
        // std::cerr << "\n\nInit with tasks: \n";
        // for (const auto& task : taskData_->taskPhases) {
        //     std::cerr << task.first << " - " << task.second << std::endl;
        // }
        std::cerr << "Next state: " << taskData_->taskPhases.front().first << std::endl;
        return fsm_->SetNextState(taskData_->taskPhases.front().first);
    }

    fsm::retval StateInit::OnExit()
    {
        return fsm::ok;
    }
}
}
