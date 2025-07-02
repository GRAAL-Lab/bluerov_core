#include "mission_ctrl/states/state_search_object.hpp"

namespace mission {

namespace states {

    StateSearchObject::StateSearchObject()
    {
    }

    StateSearchObject::~StateSearchObject() { }

    fsm::retval StateSearchObject::OnEntry()
    {
        return StartSearch();
    }

    fsm::retval StateSearchObject::Execute()
    {

        if (!systemStatus_->conf.simPerception && ctrlData->perceptionData.newDtcFromPerception) {
            ctrlData->perceptionData.newDtcFromPerception = false;
            if (taskData_->taskPhases.front().second == opis::gate) {
                for (auto& db_first : ctrlData->perceptionData.detectedBuoys) {
                    for (auto& db_second : ctrlData->perceptionData.detectedBuoys) {
                        if (db_first.second.detectionId == db_second.second.detectionId) {
                            continue; // same buoy
                        }
                        if (ctrlData->missionData.gate.SetGateBuoys(db_first.second, db_second.second, systemStatus_->conf.ignoreBuoyColor)) {
                            // Found the gate
                            if (systemStatus_->conf.debugPrints) {
                                std::cerr << "Gate found: " << db_first.second.detectionId << " and " << db_second.second.detectionId << "\n";
                            }
                            return SetNextMissionState();
                        }
                    }
                }
            } else if (taskData_->taskPhases.front().second == opis::mainPipe) {
            } else if (taskData_->taskPhases.front().second == opis::manipulationConsole) {
                if (ctrlData->perceptionData.dtcList.manipulation_console){
                    if (systemStatus_->conf.debugPrints) {
                                std::cerr << "Manipulation console FOUND!!! \n";
                            }
                    return SetNextMissionState();}
            }
        }

        if (ctrlData->kclData.kclActionCmd.feedback.actual_state != "PATH_FOLLOWING" ) {
            return StartSearch();
        }

        return fsm::ok;
    }

    fsm::retval StateSearchObject::OnExit()
    {
        ctrlData->perceptionData.enableDtcBuoys = false;
        ctrlData->perceptionData.enableDtcManipulationConsole = false;
        ctrlData->perceptionData.desiredGimbalAttitude = 0.0;

        return fsm::ok;
    }

    fsm::retval StateSearchObject::StartSearch()
    {
        if (taskData_->taskPhases.front().second == opis::gate) {
            std::cerr << "Searching for gate...\n";
            ctrlData->perceptionData.enableDtcBuoys = true;

            ctrlData->kclData.kclActionCmd = mission::kclCmd();
            ctrlData->kclData.kclActionCmd.goal.desired_state = "PATH_FOLLOWING";
            ctrlData->kclData.kclActionCmd.goal.path_mode = "Spiral2D";
            ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_diameter = 5.0;
            ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_increment = 0.5;

        } else if (taskData_->taskPhases.front().second == opis::mainPipe) {
            std::cerr << "Searching for main pipe...\n";

            // tell the KCL to turn around
        } else if (taskData_->taskPhases.front().second == opis::manipulationConsole) {
            std::cerr << "Searching for manipulation console...\n";

            ctrlData->perceptionData.enableDtcManipulationConsole = true;
            ctrlData->perceptionData.desiredGimbalAttitude = 45.0 / 180.0 * M_PI; // 50 degrees max

            ctrlData->kclData.kclActionCmd = mission::kclCmd();
            ctrlData->kclData.kclActionCmd.goal.desired_state = "PATH_FOLLOWING";
            ctrlData->kclData.kclActionCmd.goal.path_mode = "Spiral2D";
            ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_diameter = 5.0;
            ctrlData->kclData.kclActionCmd.goal.spiral_data.spiral_increment = 0.5;

            // tell the KCL to turn around
        } else {
            return fsm::fail;
        }

        return fsm::ok;
    }
}
}
