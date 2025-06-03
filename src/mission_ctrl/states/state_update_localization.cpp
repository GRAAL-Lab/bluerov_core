#include "mission_ctrl/states/state_update_localization.hpp"

namespace mission {

namespace states {

    StateUpdateLocalization::StateUpdateLocalization()
    {
    }

    StateUpdateLocalization::~StateUpdateLocalization() { }

    fsm::retval StateUpdateLocalization::OnEntry()
    {
        startingPosition.latitude = ctrlData->inertialF_linearPosition.latitude;
        startingPosition.longitude = ctrlData->inertialF_linearPosition.longitude;
        onSurface = false;
        diving = false;
        localizationStartTime = systemStatus_->lastSystemTime;

        ctrlData->kclData.newCommand = true;
        ctrlData->kclData.new_kcl_command.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.new_kcl_command.position.latitude = startingPosition.latitude;
        ctrlData->kclData.new_kcl_command.position.longitude = startingPosition.longitude;
        ctrlData->kclData.new_kcl_command.depth = taskData_->surfaceDepth;

        if (systemStatus_->conf.debugPrints) {
            std::cerr << "Surfacing for localization" << std::endl;
        }
        return fsm::ok;
    }

    fsm::retval StateUpdateLocalization::Execute()
    {
        if (!onSurface && abs(ctrlData->depth - taskData_->surfaceDepth) < systemStatus_->conf.depthTolerance) {
            onSurface = true;
            localizationStartTime = systemStatus_->lastSystemTime;
            if (systemStatus_->conf.debugPrints) {
                std::cerr << "Surfaced and waiting " << maxTimeForLocalization << " seconds for localization" << std::endl;
            }

            return fsm::ok;
        }

        auto elapsedTime = (systemStatus_->lastSystemTime - localizationStartTime).seconds();
        if (onSurface && !diving && elapsedTime > maxTimeForLocalization) {
            diving = true;
            ctrlData->kclData.newCommand = true;
            ctrlData->kclData.new_kcl_command.desired_state = "WAYPOINT_NAVIGATION";
            ctrlData->kclData.new_kcl_command.position.latitude = startingPosition.latitude;
            ctrlData->kclData.new_kcl_command.position.longitude = startingPosition.longitude;
            ctrlData->kclData.new_kcl_command.depth = taskData_->diveDepth;
            if (systemStatus_->conf.debugPrints) {
                std::cerr << "Done, diving again." << std::endl;
            }
            return fsm::ok;
        }

        if (diving && abs(ctrlData->depth - taskData_->diveDepth) < systemStatus_->conf.depthTolerance) {
            diving = false;
            onSurface = false;
            return this->SetNextMissionState();
        }

        if (systemStatus_->conf.simKcl && !onSurface) {
            ctrlData->depth = taskData_->diveDepth;
        } else if (systemStatus_->conf.simKcl && diving) {
            ctrlData->depth = taskData_->surfaceDepth;
        }
        return fsm::ok;
    }

    fsm::retval StateUpdateLocalization::OnExit()
    {
        return fsm::ok;
    }
}
}
