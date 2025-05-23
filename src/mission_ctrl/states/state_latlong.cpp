#include "mission_ctrl/states/state_latlong.hpp"

namespace mission {

namespace states {

    StateLatLong::StateLatLong()
    {
        minAcceptanceRadius = 0.5;
    }

    StateLatLong::~StateLatLong() { }

    fsm::retval StateLatLong::OnEntry()
    {
        if (taskData_->taskPhases.front().second == opis::uavWaypoint) {
            std::shared_ptr<Inspection> inspectionConf = std::dynamic_pointer_cast<Inspection>(taskData_);
            goalPose.at(0) = inspectionConf->uavWaypoint.latitude;
            goalPose.at(1) = inspectionConf->uavWaypoint.longitude;
            goalPose.at(2) = taskData_->surfaceDepth;
        } else if (taskData_->taskPhases.front().second == "DIVE") {
            goalPose.at(0) = ctrlData->inertialF_linearPosition.latitude;
            goalPose.at(1) = ctrlData->inertialF_linearPosition.longitude;
            goalPose.at(2) = taskData_->diveDepth;
        } else if (taskData_->taskPhases.front().second == "SURFACE") {
            goalPose.at(0) = ctrlData->inertialF_linearPosition.latitude;
            goalPose.at(1) = ctrlData->inertialF_linearPosition.longitude;
            goalPose.at(2) = taskData_->surfaceDepth;
        }

        std::cerr << "Moving to LatLong: (" << goalPose.at(0) << ", " << goalPose.at(1) << ") and Depth: " << goalPose.at(2) << "\n";

        ctrlData->kclData.newCommand = true;
        ctrlData->kclData.kcl_command.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kcl_command.position.latitude = goalPose.at(0);
        ctrlData->kclData.kcl_command.position.longitude = goalPose.at(1);
        ctrlData->kclData.kcl_command.depth = goalPose.at(2);

#ifdef NO_KCL
        ctrlData->inertialF_linearPosition.latitude = goalPosition.latitude;
        ctrlData->inertialF_linearPosition.longitude = goalPosition.longitude;
#endif

        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        double depthTolerance = 0.2;
        if(taskData_->taskPhases.front().second == "DIVE" || taskData_->taskPhases.front().second == "SURFACE"){
            if (abs(ctrlData->depth - goalPose.at(2)) < depthTolerance)
                {
                    return this->SetNextMissionState();
                }
        } else {
            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, { goalPose.at(0), goalPose.at(1) }, distance, azimuthRad);
            if (distance < minAcceptanceRadius) {
                return this->SetNextMissionState();
            } else {
                if (distance > 1000) {
                    std::cerr << "Distance to goal: " << distance << " (undetermined)" << "\n";
                } else {
                    std::cerr << "Distance to goal: " << distance << "\n";
                }
            }
        }

#ifdef NO_KCL
        return this->SetNextMissionState();
#endif

        return fsm::ok;
    }

    fsm::retval StateLatLong::OnExit()
    {
        return fsm::ok;
    }
}
}
