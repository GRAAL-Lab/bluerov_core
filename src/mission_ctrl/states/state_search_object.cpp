#include "mission_ctrl/states/state_search_object.hpp"

namespace mission {

namespace states {

    StateSearchObject::StateSearchObject()
    {
    }

    StateSearchObject::~StateSearchObject() { }

    fsm::retval StateSearchObject::OnEntry()
    {
        found = false;
        fsm::retval ret;
        if (taskData_->taskPhases.front().second == opis::gate) {
            ret = SearchGate();
        } else if (taskData_->taskPhases.front().second == opis::mainPipe) {
            ret = SearchMainPipe();
        } else if (taskData_->taskPhases.front().second == opis::manipulationConsole) {
            ret = SearchManipulationConsole();
        } else {
            ret = fsm::fail;
        }

        return ret;
    }

    fsm::retval StateSearchObject::Execute()
    {
        if (found) {
            std::cerr << "Found!\n";
            return this->SetNextMissionState();
        }
        double delta = std::fmod((ctrlData->bodyF_angularPosition.Yaw() - previous_bodyF_angularPosition.Yaw()) + 180, 360) - 180;
        cumulativeAngle += delta;
        previous_bodyF_angularPosition = ctrlData->bodyF_angularPosition;

        if (cumulativeAngle > 360 || cumulativeAngle < -360) {
            std::cerr << "Not Found!\n";
            // return fsm_->SetNextState(taskData_->taskPhases.front().first);
            return fsm::fail;
        }

        std::cerr << "Searching...\n";
        std::cerr << "Cumulative angle: " << cumulativeAngle << "\n";

        //temp
        found = true;

        return fsm::ok;
    }

    fsm::retval StateSearchObject::OnExit()
    {
        return fsm::ok;
    }

    fsm::retval StateSearchObject::SearchGate()
    {
        std::cerr << "Searching for gate...\n";
        previous_bodyF_angularPosition = ctrlData->bodyF_angularPosition;
        cumulativeAngle = 0;
        // tell the KCL to turn around
        return fsm::ok;
    }

    fsm::retval StateSearchObject::SearchMainPipe()
    {
        std::cerr << "Searching for main pipe...\n";
        previous_bodyF_angularPosition = ctrlData->bodyF_angularPosition;
        cumulativeAngle = 0;
        // tell the KCL to turn around
        return fsm::ok;
    }

    fsm::retval StateSearchObject::SearchManipulationConsole()
    {
        std::cerr << "Searching for manipulation console...\n";
        previous_bodyF_angularPosition = ctrlData->bodyF_angularPosition;
        cumulativeAngle = 0;
        // tell the KCL to turn around
        return fsm::ok;
    }
}
}
