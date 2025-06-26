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
        debugCounter = 0;

        if (systemStatus_->conf.debugPrints) {
            std::cerr << "\nInspecting buoy...\n";
        }

        double distanceFromBuoy = 1.0; // 1m from the buoy
        std::pair<std::string, Buoy> buoyToInspect;
        ctb::LatLong buoyPosition;

        if (systemStatus_->conf.simPerception) {
            Eigen::Vector3d buoyPositionLocal = { 10.0, 0.0, ctrlData->depth }; // Simulated position in local NED coordinates
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
            std::cerr << "      Inspecting buoy at position: " << buoyPosition.latitude << ", " << buoyPosition.longitude << std::endl;
            Eigen::Vector3d buoyPositionLocal;
            ctb::LatLong2LocalNED(buoyPosition, 0, ctrlData->inertialF_linearPosition, buoyPositionLocal);
            auto offset = -buoyPositionLocal.normalized() * distanceFromBuoy;
            buoyPositionLocal += offset;
            double alt;
            ctb::LocalNED2LatLong(buoyPositionLocal, ctrlData->inertialF_linearPosition, buoyPosition, alt);
            std::cerr << "          from position: " << buoyPosition.latitude << ", " << buoyPosition.longitude << std::endl;
        }
        // Set the goal position for the KCL command
        ctrlData->kclData.kclActionCmd = mission::kclCmd();
        ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kclActionCmd.goal.position.latitude = buoyPosition.latitude;
        ctrlData->kclData.kclActionCmd.goal.position.longitude = buoyPosition.longitude;
        ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepth; 

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::Execute()
    {
        debugCounter++;
        if (debugCounter >= 30) {
            std::cerr << "Reached buoy but actions are not implemented yet." << std::endl;

            return fsm_->SetNextState(states::ID::searchBuoyArea);
        }

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::OnExit()
    {

        return fsm::ok;
    }
}
}
