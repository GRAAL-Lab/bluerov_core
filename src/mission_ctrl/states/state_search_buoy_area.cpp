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
        debugCounter = 0;
        sentPathFollowingCommand = false;

        // tell perception to look for buoys
        ctrlData->perceptionData.enableDtcBuoys = true;

        if (resumeSearch && reachedLeftmostPoint) {
            return fsm::ok;
        }

        if (!resumeSearch) {
            //First time here
            areaPoints = std::queue<ctb::LatLong>();

            numberOfBuoysInspected = 0;
            if (taskData_->taskType == taskBenchmarks::INSPECTION) {
                std::shared_ptr<Inspection> inspectionConf = std::dynamic_pointer_cast<Inspection>(taskData_);
                numberOfBuoys = inspectionConf->numberOfBuoys;
            } else if (taskData_->taskType == taskBenchmarks::INSPECTION_AND_INTERVENTION) {
                std::shared_ptr<InspectionAndIntervention> inspectionConf = std::dynamic_pointer_cast<InspectionAndIntervention>(taskData_);
                numberOfBuoys = inspectionConf->numberOfBuoys;
            } else {
                this->SetNextMissionState();
                return fsm::ok; // No buoys to search for
            }

            // Ordering points for the area coverage path (depening on current position)
            auto points = taskData_->buoysArea.points;
            auto removePoint = [&points](const ctb::LatLong& point) {
                points.erase(
                    std::remove_if(points.begin(), points.end(),
                        [&point](const ctb::LatLong& p) {
                            return p.latitude == point.latitude && p.longitude == point.longitude;
                        }),
                    points.end());
            };
            auto findLeftmostPoint = [&](const ctb::LatLong& reference) -> std::optional<ctb::LatLong> {
                if (points.empty())
                    return std::nullopt;
                ctb::LatLong leftmostPoint = points.front();
                double minAzimuthRad = M_PI;
                for (const auto& point : points) {
                    double distance, azimuthRad;
                    ctb::DistanceAndAzimuthRad(reference, point, distance, azimuthRad);
                    if (azimuthRad < minAzimuthRad) {
                        minAzimuthRad = azimuthRad;
                        leftmostPoint = point;
                    }
                }
                return leftmostPoint;
            };
            for (size_t i = 0; i < taskData_->buoysArea.points.size(); ++i) {
                auto maybePoint = findLeftmostPoint(ctrlData->inertialF_linearPosition);
                if (!maybePoint)
                    break; // no more points
                areaPoints.push(*maybePoint);
                removePoint(*maybePoint);
            }
        }

        // Move to leftmost point
        ctrlData->kclData.kclActionCmd = mission::kclCmd();
        ctrlData->kclData.kclActionCmd.goal.desired_state = "WAYPOINT_NAVIGATION";
        ctrlData->kclData.kclActionCmd.goal.position.latitude = areaPoints.front().latitude;
        ctrlData->kclData.kclActionCmd.goal.position.longitude = areaPoints.front().longitude;
        ctrlData->kclData.kclActionCmd.goal.depth = systemStatus_->conf.diveDepth; 
        return fsm::ok;
    }

    fsm::retval StateSearchBuoyArea::Execute()
    {
        if (numberOfBuoysInspected >= numberOfBuoys) {
            // tell kcl to stop following area coverage path
            // tell perception to stop looking for buoys
            ctrlData->perceptionData.enableDtcBuoys = false;
            resumeSearch = false;
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
            resumeSearch = true;
            // tell kcl to inspect buoy
            if (systemStatus_->conf.debugPrints)
                std::cerr << "\nGoing to inspect a buoy." << std::endl;

            numberOfBuoysInspected++;
            return fsm_->SetNextState(states::ID::inspectBuoy);
        }

        if (!reachedLeftmostPoint) {
            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, areaPoints.front(), distance, azimuthRad);
            if (distance < systemStatus_->conf.latlongTolerance) {
                reachedLeftmostPoint = true;
            }
            return fsm::ok;
        }

        if (reachedLeftmostPoint && !sentPathFollowingCommand) {
            sentPathFollowingCommand = true;
            
            // tell kcl to follow area coverage path
            ctrlData->kclData.kclActionCmd = mission::kclCmd();
            if (resumeSearch) {
                ctrlData->kclData.kclActionCmd.goal.resume_path = true;
            } 

            ctrlData->kclData.kclActionCmd.goal.desired_state = "PATH_FOLLOWING";
            ctrlData->kclData.kclActionCmd.goal.path_mode = "Serpentine2D";

            auto points = areaPoints;

            ctrlData->kclData.kclActionCmd.goal.serpentine_data.origin.latitude = points.front().latitude;
            ctrlData->kclData.kclActionCmd.goal.serpentine_data.origin.longitude = points.front().longitude;
            points.pop();
            ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_left.longitude = points.front().longitude;
            ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_left.latitude = points.front().latitude;
            points.pop();
            ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_right.longitude = points.front().longitude;
            ctrlData->kclData.kclActionCmd.goal.serpentine_data.front_right.latitude = points.front().latitude;
            points.pop();
            ctrlData->kclData.kclActionCmd.goal.serpentine_data.right.longitude = points.front().longitude;
            ctrlData->kclData.kclActionCmd.goal.serpentine_data.right.latitude = points.front().latitude;
            return fsm::ok;
        }

        debugCounter++;
        if (systemStatus_->conf.simPerception && debugCounter >= 160) {
            std::cerr << "Simulating perception, mock up buoy search." << std::endl;
            resumeSearch = true;
            numberOfBuoysInspected++;
            return fsm_->SetNextState(states::ID::inspectBuoy);
        }

        return fsm::ok;
    }

    fsm::retval StateSearchBuoyArea::OnExit()
    {
        return fsm::ok;
    }
}
}
