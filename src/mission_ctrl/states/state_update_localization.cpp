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

        ctrlData->kclData.kclActionCmd = mission::kclCmd();
        ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kclActionCmd.goal.position.latitude = startingPosition.latitude;
        ctrlData->kclData.kclActionCmd.goal.position.longitude = startingPosition.longitude;
        ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepth;

        if (systemStatus_->conf.debugPrints) {
            std::cerr << "Surfacing for localization" << std::endl;
        }
        return fsm::ok;
    }

    fsm::retval StateUpdateLocalization::Execute()
    {
        if (!onSurface && abs(ctrlData->depth - systemStatus_->conf.surfaceDepth) < systemStatus_->conf.depthTolerance) {
            onSurface = true;
            localizationStartTime = std::chrono::steady_clock::now();
            if (systemStatus_->conf.debugPrints) {
                std::cerr << "Surfaced and waiting " << maxTimeForLocalization << " seconds for localization" << std::endl;
            }

            return fsm::ok;
        }

        auto elapsedTime = (std::chrono::steady_clock::now() - localizationStartTime).count() / 1e9; // Convert to seconds
        if (onSurface && !diving && elapsedTime > maxTimeForLocalization) {
            diving = true;
            ctrlData->kclData.kclActionCmd = mission::kclCmd();
            ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
            ctrlData->kclData.kclActionCmd.goal.position.latitude = startingPosition.latitude;
            ctrlData->kclData.kclActionCmd.goal.position.longitude = startingPosition.longitude;
            ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepth;
            if (systemStatus_->conf.debugPrints) {
                std::cerr << "Done, diving again." << std::endl;
            }
            return fsm::ok;
        }

        if (diving && abs(ctrlData->depth - systemStatus_->conf.diveDepth) < systemStatus_->conf.depthTolerance) {
            diving = false;
            onSurface = false;
            return this->SetNextMissionState();
        }

        if (systemStatus_->conf.simKcl && !onSurface) {
            ctrlData->depth = systemStatus_->conf.diveDepth;
        } else if (systemStatus_->conf.simKcl && diving) {
            ctrlData->depth = systemStatus_->conf.surfaceDepth;
        }
        return fsm::ok;
    }

    fsm::retval StateUpdateLocalization::OnExit()
    {
        return fsm::ok;
    }
}
}
