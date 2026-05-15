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
            systemStatus_->SetState(MissionCtrlState::WAITING_FOR_SYSTEM_TO_BE_READY);
            return fsm::ok;
        } else {
            if (systemStatus_->conf.debugPrints)
                std::cerr << "\nNext state: " << taskData_->taskPhases.front().first << std::endl;
            return fsm_->SetNextState(taskData_->taskPhases.front().first);
        }
    }


}
}
