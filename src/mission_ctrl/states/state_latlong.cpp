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
        ctrlData->kclData.kcl_command.latitude = goalPosition.latitude;
        ctrlData->kclData.kcl_command.longitude = goalPosition.longitude;

        // ctrlData->inertialF_linearPosition.latitude = goalPosition.latitude;
        // ctrlData->inertialF_linearPosition.longitude = goalPosition.longitude;

        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        double alt;
        Eigen::Vector3d distanceVector;
        ctb::LatLong2LocalNED(ctrlData->inertialF_linearPosition, alt, goalPosition, distanceVector);
        if (distanceVector.norm() < minAcceptanceRadius) {
            this->SetNextMissionState();
        } else {
            std::cerr << "Distance to goal: " << distanceVector.norm() << "\n";
        }

#ifdef DEBUG
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
