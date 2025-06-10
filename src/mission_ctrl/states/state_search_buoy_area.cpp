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

        // tell perception to look for buoys
        ctrlData->perceptionData.enableDtcBuoys = true;

        // tell kcl to follow area coverage path
        ctrlData->kclData.newCommand = true;
        ctrlData->kclData.new_kcl_command.desired_state = "PATH_FOLLOWING";
        ctrlData->kclData.new_kcl_command.path_mode = "Serpentine2D";

        if (resumeSearch) {
            ctrlData->kclData.new_kcl_command.resume_path = true;
            return fsm::ok;
        } else {
            numberOfBuoysInspected = 0;
            ctrlData->kclData.new_kcl_command.resume_path = false;
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
        }

        auto points = taskData_->buoysArea.points;
        auto removePoint = [&points](const ctb::LatLong& point) {
            points.erase(
                std::remove_if(points.begin(), points.end(),
                    [&point](const ctb::LatLong& p) {
                        return p.latitude == point.latitude && p.longitude == point.longitude;
                    }),
                points.end());
        };
        // Get first point as the closest to current position
        ctb::LatLong firstPoint;
        double minDistance = std::numeric_limits<double>::max();
        for (const auto& point : points) {
            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, point, distance, azimuthRad);
            if (distance < minDistance) {
                minDistance = distance;
                firstPoint = point;
            }
        }
        removePoint(firstPoint);
        ctrlData->kclData.new_kcl_command.serpentine_data.origin.latitude = firstPoint.latitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.origin.longitude = firstPoint.longitude;
        // Get the leftmost from the remaining points
        ctb::LatLong leftmostPoint = points[0];
        double minAzimuthRad = M_PI;
        for (const auto& point : points) {
            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, point, distance, azimuthRad);
            if (azimuthRad < minAzimuthRad || minAzimuthRad == 0.0) {
                minAzimuthRad = azimuthRad;
                leftmostPoint = point;
            }
        }
        removePoint(leftmostPoint);
        // Get the rightmost from the remaining points
        ctb::LatLong rightmostPoint = points[0];
        double maxAzimuthRad = -M_PI;
        for (const auto& point : points) {
            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, point, distance, azimuthRad);
            if (azimuthRad > maxAzimuthRad || maxAzimuthRad == 0.0) {
                maxAzimuthRad = azimuthRad;
                rightmostPoint = point;
            }
        }
        removePoint(rightmostPoint);

        ctrlData->kclData.new_kcl_command.serpentine_data.origin.latitude = firstPoint.latitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.origin.longitude = firstPoint.longitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.front_left.latitude = leftmostPoint.latitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.front_left.longitude = leftmostPoint.longitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.right.latitude = rightmostPoint.latitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.right.longitude = rightmostPoint.longitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.front_right.latitude = points[0].latitude;
        ctrlData->kclData.new_kcl_command.serpentine_data.front_right.longitude = points[0].longitude;

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

        if (systemStatus_->conf.simPerception) {
            std::cerr << "Simulating perception, mock up buoy search." << std::endl;
            resumeSearch = true;
            numberOfBuoysInspected++;
            return fsm_->SetNextState(states::ID::inspectBuoy);
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

        return fsm::ok;
    }

    fsm::retval StateSearchBuoyArea::OnExit()
    {

        return fsm::ok;
    }
}
}
