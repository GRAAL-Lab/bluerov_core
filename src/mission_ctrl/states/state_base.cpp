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
        std::cerr << "\nNext state: " << taskData_->taskPhases.front().first << std::endl;
        
        return fsm_->SetNextState(taskData_->taskPhases.front().first);
    }

}
}
