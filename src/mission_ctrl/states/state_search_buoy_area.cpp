#include "mission_ctrl/states/state_search_buoy_area.hpp"

namespace mission {

namespace states {

    StateSearchBuoyArea::StateSearchBuoyArea()
    {
    }

    StateSearchBuoyArea::~StateSearchBuoyArea()
    {
    }

    fsm::retval StateSearchBuoyArea::OnEntry()
    {

        numberOfBuoysInspected = 0;
        if (taskData_->taskType == taskBenchmarks::INSPECTION) {
            std::shared_ptr<Inspection> inspectionConf = std::dynamic_pointer_cast<Inspection>(taskData_);
            numberOfBuoys = inspectionConf->numberOfBuoys;
        } else if (taskData_->taskType == taskBenchmarks::INSPECTION_AND_INTERVENTION) {
            std::shared_ptr<InspectionAndIntervention> inspectionConf = std::dynamic_pointer_cast<InspectionAndIntervention>(taskData_);
            numberOfBuoys = inspectionConf->numberOfBuoys;
        } // else{
        //     // state is created but not used
        // }

        // tell kcl to follow area coverage path

        // tell perception to look for buoys

        return fsm::ok;
    }

    fsm::retval StateSearchBuoyArea::Execute()
    {

        if (numberOfBuoysInspected >= numberOfBuoys) {
            // tell kcl to stop following area coverage path
            // tell perception to stop looking for buoys
            return this->SetNextMissionState();
        }

        for (auto& db : ctrlData->perceptionData.detectedBuoys) {
            for (auto& ib : ctrlData->missionData.inspectedBuoys) {
                if (db.second.detectionId == ib.detectionId) {
                    // already inspected
                    continue;
                }
            }

            // save path so far
            // tell kcl to inspect buoy

            numberOfBuoysInspected++;
            break;
        }

        return fsm::ok;
    }

    fsm::retval StateSearchBuoyArea::OnExit()
    {

        return fsm::ok;
    }
}
}
