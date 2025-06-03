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
            if (systemStatus_->conf.debugPrints)
                std::cerr << "No more task phases to execute, back to init." << std::endl;
            return fsm_->SetNextState(states::ID::init);
        } else {
            if (systemStatus_->conf.debugPrints)
                std::cerr << "\nNext state: " << taskData_->taskPhases.front().first << std::endl;
            return fsm_->SetNextState(taskData_->taskPhases.front().first);
        }
    }

}
}
