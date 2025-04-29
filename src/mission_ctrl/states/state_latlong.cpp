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
        if(taskData_->taskPhases.back().second == opis::uavWaypoint){
            std::shared_ptr<Inspection> inspectionConf = std::dynamic_pointer_cast<Inspection>(taskData_);
            goalPosition = inspectionConf->uavWaypoint;
        }

        std::cerr << "Moving to: " << taskData_->taskPhases.back().second << " at " << goalPosition.latitude << ", "
                  << goalPosition.longitude << std::endl;
        // tell kcl to reach goalPosition

        ctrlData->inertialF_linearPosition.latitude = goalPosition.latitude;
        ctrlData->inertialF_linearPosition.longitude = goalPosition.longitude;

        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        double alt;
        Eigen::Vector3d distanceVector;
        ctb::LatLong2LocalNED(ctrlData->inertialF_linearPosition, alt, goalPosition, distanceVector);
        if (distanceVector.norm() < minAcceptanceRadius) {
            taskData_->taskPhases.pop();
            std::cerr << "Reached!\n";
            return fsm_->SetNextState(taskData_->taskPhases.front().first);
        }else{
            std::cerr << "Distance to goal: " << distanceVector.norm() << "\n";
        }

        return fsm::ok;
    }

    fsm::retval StateLatLong::OnExit()
    {
        return fsm::ok;
    }
}
}
