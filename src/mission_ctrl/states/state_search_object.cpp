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
        // if(ctrlData->perceptionData.newDtcFromPerception){
        //     ctrlData->perceptionData.newDtcFromPerception = false;
        //     if (taskData_->taskPhases.front().second == opis::gate) {
        //         for (auto& db_first : ctrlData->perceptionData.detectedBuoys) {
        //             for (auto& db_second : ctrlData->perceptionData.detectedBuoys){
        //                 if (db_first.second.detectionId == db_second.second.detectionId) {
        //                     continue; // same buoy
        //                 }
        //                 if(gate.SetGateBuoys(db_first.second, db_second.second)) {
        //                     // Found the gate
        //                     if (systemStatus_->conf.debugPrints) {
        //                         std::cerr << "Gate found: " << db_first.second.detectionId << " and " << db_second.second.detectionId << "\n";
        //                     }
        //                     found = true;
        //                     break; // exit inner loop
        //                 }
        //             }

        //         }
                
        //     } else if (taskData_->taskPhases.front().second == opis::mainPipe) {
                
        //     } else if (taskData_->taskPhases.front().second == opis::manipulationConsole) {
               
        //     }
        // }

        // if (found) {
        //     std::cerr << "Found!\n";
        //     return this->SetNextMissionState();
        // }
        double delta = std::fmod((ctrlData->bodyF_angularPosition.Yaw() - previous_bodyF_angularPosition.Yaw()) + 180, 360) - 180;
        cumulativeAngle += delta;
        previous_bodyF_angularPosition = ctrlData->bodyF_angularPosition;

        // if (cumulativeAngle > 360 || cumulativeAngle < -360) {
        //     std::cerr << "Not Found!\n";
        //     // return fsm_->SetNextState(taskData_->taskPhases.front().first);
        //     return fsm::fail;
        // }

        if (systemStatus_->conf.debugPrints) {
            //std::cerr << ".";
            std::cerr << "      Cumulative angle: " << cumulativeAngle << "\n";
        }

        // if (systemStatus_->conf.simPerception)
        //     found = true;

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

        ctrlData->kclData.newCommand = true;
        ctrlData->kclData.new_kcl_command.desired_state = "PATH_FOLLOWING";
        ctrlData->kclData.new_kcl_command.path_mode = "Spiral2D";
        ctrlData->kclData.new_kcl_command.spiral_data.spiral_diameter = 10.0; 
        ctrlData->kclData.new_kcl_command.spiral_data.spiral_increment = 1.0;

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
