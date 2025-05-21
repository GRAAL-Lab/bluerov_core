#include "mission_ctrl/states/state_homing.hpp"

namespace mission {

namespace states {

    StateHoming::StateHoming()
    {
        minAcceptanceRadius = 0.5;
    }

    StateHoming::~StateHoming() { }

    fsm::retval StateHoming::OnEntry()
    {

        std::cerr << "Moving home: " << homePosition.latitude << ", "
                  << homePosition.longitude << std::endl;

        ctrlData->inertialF_linearPosition.latitude = homePosition.latitude;
        ctrlData->inertialF_linearPosition.longitude = homePosition.longitude;

        return fsm::ok;
    }

    fsm::retval StateHoming::Execute()
    {
        double distance, azimuthRad;
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, homePosition, distance, azimuthRad);
        if (distance < minAcceptanceRadius) {
            std::cerr << "Reached home!\n";
            return fsm_->SetNextState(states::ID::init);
        } else {
            std::cerr << "Distance to home: " << distance << "\n";
        }

        return fsm::ok;
    }

    fsm::retval StateHoming::OnExit()
    {
        return fsm::ok;
    }
}
}
