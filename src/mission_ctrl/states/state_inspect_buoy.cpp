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
        //std::pair<std::string, Buoy> buoyToInspect;

        if (systemStatus_->conf.simPerception) {
            Eigen::Vector3d buoyPositionLocal = { 10.0, 0.0, ctrlData->depth }; // Simulated position in local NED coordinates
            double alt;
            ctb::LocalNED2LatLong(buoyPositionLocal, ctrlData->inertialF_linearPosition, inspectionPosition, alt);
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
                    //buoyToInspect = db;
                    buoyToInspect = db.second;
                    break;
                }
            }

            // Get point at 1m from the buoy in out direction
            inspectionPosition = buoyToInspect.position;
            std::cerr << "      Inspecting buoy at position: " << buoyToInspect.position.latitude << ", " << buoyToInspect.position.longitude << std::endl;
            Eigen::Vector3d buoyPositionLocal;
            ctb::LatLong2LocalNED(buoyToInspect.position, 0, ctrlData->inertialF_linearPosition, buoyPositionLocal);
            auto offset = -buoyPositionLocal.normalized() * distanceFromBuoy;
            buoyPositionLocal += offset;
            double alt;
            ctb::LocalNED2LatLong(buoyPositionLocal, ctrlData->inertialF_linearPosition, inspectionPosition, alt);
            std::cerr << "          from position: " << inspectionPosition.latitude << ", " << inspectionPosition.longitude << std::endl;
        }
        // Set the goal position for the KCL command
        ctrlData->kclData.kclActionCmd = mission::kclCmd();
        ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kclActionCmd.goal.position.latitude = inspectionPosition.latitude;
        ctrlData->kclData.kclActionCmd.goal.position.longitude = inspectionPosition.longitude;
        ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepthBuoys;

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::Execute()
    {

        double distance, azimuthRad;
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, inspectionPosition, distance, azimuthRad);
        if (distance < systemStatus_->conf.latlongTolerance) {
            std::cerr << "Reached buoy but actions are not implemented yet." << std::endl;

            ctrlData->kclData.kclActionCmd = mission::kclCmd();
        ctrlData->kclData.kclActionCmd.goal.desired_state = "Circular2D";
        ctrlData->kclData.kclActionCmd.goal.circular_data.circular_diameter = 1.0; // 1m radius
        ctrlData->kclData.kclActionCmd.goal.circular_data.center_point.latitude = buoyToInspect.position.latitude;
        ctrlData->kclData.kclActionCmd.goal.circular_data.center_point.longitude = buoyToInspect.position.longitude;
        ctrlData->kclData.kclActionCmd.goal.circular_data.clockwise = true; // Clockwise rotation

            //return fsm_->SetNextState(states::ID::searchBuoyArea);
        }

        return fsm::ok;
    }

    fsm::retval StateInspectBuoy::OnExit()
    {

        return fsm::ok;
    }
}
}
