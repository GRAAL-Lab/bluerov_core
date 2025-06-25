#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    StateBase::StateBase()
    {
    }

    StateBase::~StateBase()
    {
    }

    fsm::retval StateBase::SetNextMissionState()
    {
        taskData_->taskPhases.pop();
        if (taskData_->taskPhases.empty()) {
            return StopMission();
        } else {
            if (systemStatus_->conf.debugPrints)
                std::cerr << "\nNext state: " << taskData_->taskPhases.front().first << std::endl;
            return fsm_->SetNextState(taskData_->taskPhases.front().first);
        }
    }

    fsm::retval StateBase::StopMission()
    {
        if (systemStatus_->conf.debugPrints)
            std::cerr << "Stopping mission, back to init." << std::endl;
        return fsm_->SetNextState(states::ID::init);
    }

}
}
