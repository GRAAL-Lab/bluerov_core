#include "mission_ctrl/states/state_latlong.hpp"

namespace mission {

namespace states {

    StateLatLong::StateLatLong()
    {
    }

    StateLatLong::~StateLatLong() { }

    fsm::retval StateLatLong::OnEntry()
    {
        doneInit = false;
        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        if (!doneInit) {
            if (taskData_->taskPhases.front().second == opis::uavWaypoint) {
                std::shared_ptr<Inspection> inspectionConf = std::dynamic_pointer_cast<Inspection>(taskData_);
                goalPose.at(0) = inspectionConf->uavWaypoint.latitude;
                goalPose.at(1) = inspectionConf->uavWaypoint.longitude;
                goalPose.at(2) = systemStatus_->conf.diveDepth;
            } else if (taskData_->taskPhases.front().second == "DIVE") {
                goalPose.at(0) = ctrlData->inertialF_linearPosition.latitude;
                goalPose.at(1) = ctrlData->inertialF_linearPosition.longitude;
                goalPose.at(2) = systemStatus_->conf.diveDepth;
            } else if (taskData_->taskPhases.front().second == "SURFACE") {
                goalPose.at(0) = ctrlData->inertialF_linearPosition.latitude;
                goalPose.at(1) = ctrlData->inertialF_linearPosition.longitude;
                goalPose.at(2) = systemStatus_->conf.surfaceDepth;
            } else if (taskData_->taskPhases.front().second == "DEBUG") {
                if (systemStatus_->conf.debug_position_selection == 0 || systemStatus_->conf.debugPositions.size() < systemStatus_->conf.debug_position_selection) {
                    std::cerr << "Debug waypoint requested, but no debug positions are configured OR 0 was selected. Skipping.\n";
                    return this->SetNextMissionState();
                }
                std::cerr << "Debug waypoint requested, using debug position" << systemStatus_->conf.debug_position_selection - 1 << ": ";
                std::cerr << systemStatus_->conf.debugPositions[systemStatus_->conf.debug_position_selection - 1].latitude << ", "
                          << systemStatus_->conf.debugPositions[systemStatus_->conf.debug_position_selection - 1].longitude << ", "
                          << systemStatus_->conf.diveDepth << "\n";
                goalPose.at(0) = systemStatus_->conf.debugPositions[0].latitude;
                goalPose.at(1) = systemStatus_->conf.debugPositions[0].longitude;
                goalPose.at(2) = systemStatus_->conf.diveDepth;
            }

            if (systemStatus_->conf.debugPrints) {
                std::cerr << "Moving from LatLong: ("
                          << ctrlData->inertialF_linearPosition.latitude << ", "
                          << ctrlData->inertialF_linearPosition.longitude << ") and Depth: "
                          << ctrlData->depth << "\n";
                std::cerr << "         to LatLong: ("
                          << goalPose.at(0) << ", "
                          << goalPose.at(1) << ") and Depth: "
                          << goalPose.at(2) << "\n";
            }

            ctrlData->kclData.kclActionCmd = mission::kclCmd();
            ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
            ctrlData->kclData.kclActionCmd.goal.position.latitude = goalPose.at(0);
            ctrlData->kclData.kclActionCmd.goal.position.longitude = goalPose.at(1);
            ctrlData->kclData.kclActionCmd.goal.depth = goalPose.at(2);

            doneInit = true;
            return fsm::ok;
        }

        if (taskData_->taskPhases.front().second == "DIVE" || taskData_->taskPhases.front().second == "SURFACE") {
            if (abs(ctrlData->depth - goalPose.at(2)) < systemStatus_->conf.depthTolerance) {
                doneInit = false;
                return this->SetNextMissionState();
            }
            // if (systemStatus_->conf.debugPrints)
            //     std::cerr << "Depth to goal: " << abs(ctrlData->depth - goalPose.at(2)) << "\n";

        } else {
            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, { goalPose.at(0), goalPose.at(1) }, distance, azimuthRad);
            if (distance < systemStatus_->conf.latlongTolerance) {
                doneInit = false;
                return this->SetNextMissionState();
            }
            std::cerr << "Distance to goal: " << distance << "\n";
            // if (distance > 1000 && systemStatus_->conf.debugPrints) {
            //     std::cerr << "Distance to goal: " << distance << " (undetermined)" << "\n";
            // } else {
            //     std::cerr << "Distance to goal: " << distance << "\n";
            // }
        }

        return fsm::ok;
    }

    fsm::retval StateLatLong::OnExit()
    {
        doneInit = false;
        return fsm::ok;
    }
}
}
