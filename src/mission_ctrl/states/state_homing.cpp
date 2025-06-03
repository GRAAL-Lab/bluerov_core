#include "mission_ctrl/states/state_homing.hpp"

namespace mission {

namespace states {

    StateHoming::StateHoming()
    {
    }

    StateHoming::~StateHoming() { }

    fsm::retval StateHoming::OnEntry()
    {
        reachedSurface = false;
        if (systemStatus_->conf.debugPrints)
            std::cerr << "Surfacing and moving home: " << homePosition.latitude << ", "
                      << homePosition.longitude << std::endl;

        // Surface cmd
        ctrlData->kclData.newCommand = true;
        ctrlData->kclData.new_kcl_command.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.new_kcl_command.position.latitude = ctrlData->inertialF_linearPosition.latitude;
        ctrlData->kclData.new_kcl_command.position.longitude = ctrlData->inertialF_linearPosition.longitude;
        ctrlData->kclData.new_kcl_command.depth = taskData_->surfaceDepth;

        return fsm::ok;
    }

    fsm::retval StateHoming::Execute()
    {

        if (!reachedSurface && std::abs(ctrlData->depth - taskData_->surfaceDepth) > systemStatus_->conf.depthTolerance) {
            if (systemStatus_->conf.debugPrints)
                std::cerr << "Current/Goal depth: " << ctrlData->depth << " / " << taskData_->surfaceDepth << "\n";
            if (systemStatus_->conf.simKcl) {
                // Simulated KCL, set the position directly
                ctrlData->depth = taskData_->surfaceDepth;
            }
            return fsm::ok;
        } else if (!reachedSurface) {
            reachedSurface = true;
            ctrlData->kclData.newCommand = true;
            ctrlData->kclData.new_kcl_command.desired_state = "WAYPOINT_NAVIGATION";
            ctrlData->kclData.new_kcl_command.position.latitude = homePosition.latitude;
            ctrlData->kclData.new_kcl_command.position.longitude = homePosition.longitude;
            ctrlData->kclData.new_kcl_command.depth = taskData_->surfaceDepth;
            if (systemStatus_->conf.debugPrints)
                std::cerr << "Reached surface, moving home: " << homePosition.latitude << ", "
                          << homePosition.longitude << "\n";
            if (systemStatus_->conf.simKcl) {
                // Simulated KCL, set the position directly
                ctrlData->inertialF_linearPosition.latitude = homePosition.latitude;
                ctrlData->inertialF_linearPosition.longitude = homePosition.longitude;
            }
            return fsm::ok;
        }

        double distance, azimuthRad;
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, homePosition, distance, azimuthRad);
        if (distance < systemStatus_->conf.latlongTolerance) {
            if (systemStatus_->conf.debugPrints)
                std::cerr << "Reached home!\n";
            return this->SetNextMissionState();
        } else {
            if (systemStatus_->conf.debugPrints)
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
