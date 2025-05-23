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
            goalPosition = inspectionConf->uavWaypoint;
        }

        std::cerr << "Moving to: " << taskData_->taskPhases.front().second << " at " << goalPosition.latitude << ", "
                  << goalPosition.longitude << std::endl;

        ctrlData->kclData.newCommand = true;
        ctrlData->kclData.kcl_command.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kcl_command.position.latitude = goalPosition.latitude;
        ctrlData->kclData.kcl_command.position.longitude = goalPosition.longitude;

#ifdef NO_KCL
        ctrlData->inertialF_linearPosition.latitude = goalPosition.latitude;
        ctrlData->inertialF_linearPosition.longitude = goalPosition.longitude;
#endif

        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {

        double distance, azimuthRad;
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, goalPosition, distance, azimuthRad);
        if (distance < minAcceptanceRadius) {
            this->SetNextMissionState();
        } else {
            if(distance > 1000) {
                std::cerr << "Distance to goal: " << distance << " (undetermined)" << "\n";
            } else {
                std::cerr << "Distance to goal: " << distance << "\n";
            }
        }

#ifdef NO_KCL
        this->SetNextMissionState();
#endif

        return fsm::ok;
    }

    fsm::retval StateLatLong::OnExit()
    {
        return fsm::ok;
    }
}
}
