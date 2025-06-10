#include "mission_ctrl/states/state_inspect_buoy.hpp"

namespace mission {

namespace states {

    StateInspectBuoy::StateInspectBuoy()
    {
    }

    StateInspectBuoy::~StateInspectBuoy()
    {
    }

    fsm::retval StateInspectBuoy::OnEntry()
    {   
        if (systemStatus_->conf.debugPrints) {
            std::cerr << "\nInspecting buoy...\n";
        }

        double distanceFromBuoy = 1.0; // 1m from the buoy
        std::pair<std::string, Buoy> buoyToInspect;
        ctb::LatLong buoyPosition;
        
        if (systemStatus_->conf.simPerception) {
            Eigen::Vector3d buoyPositionLocal = {15.0, 0.0, ctrlData->depth}; // Simulated position in local NED coordinates
            double alt;
            ctb::LocalNED2LatLong(buoyPositionLocal, ctrlData->inertialF_linearPosition, buoyPosition, alt);
        } else {

            // Get the buoy to inspect from the ctrlData
            for (const auto& db : ctrlData->perceptionData.detectedBuoys) {
                bool alreadyInspected = false;
                for (const auto& ib : ctrlData->missionData.inspectedBuoys) {
                    if (db.second.detectionId == ib.detectionId) {
                        alreadyInspected = true;
                        break;
                    }
                }
                if (!alreadyInspected) {
                    buoyToInspect = db;
                    break;
                }
            }

            // Get point at 1m from the buoy in out direction
            buoyPosition = buoyToInspect.second.position;
            Eigen::Vector3d buoyPositionLocal;
            ctb::LatLong2LocalNED(buoyPosition, 0, ctrlData->inertialF_linearPosition, buoyPositionLocal);
            auto offset = -buoyPositionLocal.normalized() * distanceFromBuoy;
            buoyPositionLocal += offset;
            double alt;
            ctb::LocalNED2LatLong(buoyPositionLocal, ctrlData->inertialF_linearPosition, buoyPosition, alt);
        }
        // Set the goal position for the KCL command
        ctrlData->kclData.newCommand = true;
        ctrlData->kclData.new_kcl_command.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.new_kcl_command.position.latitude = buoyPosition.latitude;
        ctrlData->kclData.new_kcl_command.position.longitude = buoyPosition.longitude;
        ctrlData->kclData.new_kcl_command.depth = taskData_->diveDepth;

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::Execute()
    {
        std::cerr << "Reached buoy but actions are not implemented yet." << std::endl;

        return fsm_->SetNextState(states::ID::searchBuoyArea);

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::OnExit()
    {

        return fsm::ok;
    }
}
}
