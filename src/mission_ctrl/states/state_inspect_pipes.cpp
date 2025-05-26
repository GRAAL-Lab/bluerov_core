#include "mission_ctrl/states/state_inspect_pipes.hpp"

namespace mission {

namespace states {

    StateInspectPipes::StateInspectPipes()
    {
    }

    StateInspectPipes::~StateInspectPipes()
    {
    }

    fsm::retval StateInspectPipes::OnEntry()
    {

        currentPhase = PipelinePipeInspectionPhase::LOOKING_FOR_PIPE;

        if (systemStatus_->conf.debugPrints) {
            std::cerr << "Inspecting pipes...\n";
        }

        // tell kcl to spiral around the pipeline structure

        ctrlData->perceptionData.enableDtcPipes = true;
        // tell perception to detect pipes with code "" (no code)
        // ctrlData->perceptionData.currentPipeDtc.codeDtc = "";

        return fsm::ok;
    }

    // string pipeline_pipe_code       #empty, A or B. (depending on pp orientations given by the bros)
    // bool pipe_in_fov                #spotted a pipe!
    // LatLong point_on_pipe           #try to keep this constant unless it is not on the pipe anymore
    // float64 vertical_distance       #robot-pipe distance on z-axis
    // float64 pipe_direction          #NED
    // bool move_toward_structure      #this will be false at start, true once reached the end of the pipe (opposite to structure)

    // bool found_red_marker
    // Position red_marker_position
    // bool found_number
    // Position number_position
    // uint8 number # number spotted on current pipe

    fsm::retval StateInspectPipes::Execute()
    {
        std::cerr << ".";
        if (!ctrlData->perceptionData.newDtcFromPerception)
            return fsm::ok;
        ctrlData->perceptionData.newDtcFromPerception = false;

        if (currentPhase == PipelinePipeInspectionPhase::LOOKING_FOR_PIPE) {
            if (ctrlData->perceptionData.currentPipeDtc.pipe_in_fov) {
                // Found a pipe
                if (systemStatus_->conf.debugPrints) {
                    std::cerr << "Pipe found: " << ctrlData->perceptionData.currentPipeDtc.pipeline_pipe_code << "\n";
                }
                onPipeInspection = true;
                currentPipeDtcCode = ctrlData->perceptionData.currentPipeDtc.pipeline_pipe_code;
                currentPhase = PipelinePipeInspectionPhase::MOVING_TO_POINT_ON_PIPE;
            } else {
                // keep following the path around the structure
            }
        } else if (currentPhase == PipelinePipeInspectionPhase::MOVING_TO_POINT_ON_PIPE) {
            // Got a goal from dtc which should be on the pipe
            // -> check if reached the goal already
            // -> check if kcl is already moving to a point close to the new one

            ctb::LatLong pointOnPipe;
            pointOnPipe.latitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.latitude;
            pointOnPipe.longitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.longitude;

            double distance, azimuthRad;
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pointOnPipe, distance, azimuthRad);
            if (distance < 0.5) {
                // Point reached
                if (systemStatus_->conf.debugPrints) {
                    std::cerr << "Reached point on pipe: (" << pointOnPipe.latitude << ", " << pointOnPipe.longitude << ")\n";
                }
                currentPhase = PipelinePipeInspectionPhase::MOVING_AWAY_FROM_PIPELINE_STRUCTURE;
                return fsm::ok;
            }

            ctb::LatLong currentGoal;
            currentGoal.latitude = ctrlData->kclData.kcl_command.position.latitude;
            currentGoal.longitude = ctrlData->kclData.kcl_command.position.longitude;
            ctb::DistanceAndAzimuthRad(currentGoal, pointOnPipe, distance, azimuthRad);
            if (ctrlData->kclData.executingCommand && distance < 0.5) {
                return fsm::ok;
            }

            // tell kcl to move to point on pipe and align to direction
            ctrlData->perceptionData.currentPipeDtc.pipe_direction;
            ctrlData->kclData.newCommand = true;
            ctrlData->kclData.kcl_command.desired_state = "WAYPOINT_NAVIGATION";
            ctrlData->kclData.kcl_command.position.latitude = pointOnPipe.latitude;
            ctrlData->kclData.kcl_command.position.longitude = pointOnPipe.longitude;
            ctrlData->kclData.kcl_command.depth = taskData_->diveDepth;

        } else if (currentPhase == PipelinePipeInspectionPhase::MOVING_AWAY_FROM_PIPELINE_STRUCTURE) {
            if (ctrlData->perceptionData.currentPipeDtc.move_toward_structure) {
                currentPhase = PipelinePipeInspectionPhase::MOVING_TO_PIPELINE_STRUCTURE;
                auto& pipe = (currentPipeDtcCode == "A") ? pipeA : pipeB;
                pipe.endPosition.latitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.latitude;
                pipe.endPosition.longitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.longitude;
                return fsm::ok;
            }
            // tell kcl to move away from the pipeline structure
            ctrlData->perceptionData.currentPipeDtc.vertical_distance; // has to be like 0.3m

        } else if (currentPhase == PipelinePipeInspectionPhase::MOVING_TO_PIPELINE_STRUCTURE) {
            if (!ctrlData->perceptionData.currentPipeDtc.move_toward_structure) {
                // reached the structure
                if (systemStatus_->conf.debugPrints) {
                    std::cerr << "Reached pipeline structure\n";
                }
                auto& pipe = (currentPipeDtcCode == "A") ? pipeA : pipeB;
                pipe.startPosition.latitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.latitude;
                pipe.startPosition.longitude = ctrlData->perceptionData.currentPipeDtc.point_on_pipe.longitude;
                pipe.numberOfLaps++;
                currentPhase = PipelinePipeInspectionPhase::ADDITIONAL_INSPECTION_LAP;
                if (pipe.numberOfLaps >= maxInspectionlaps) {
                    onPipeInspection = false;
                    auto& otherPipe = (currentPipeDtcCode == "A") ? pipeB : pipeA;
                    if (otherPipe.numberOfLaps < maxInspectionlaps) {
                        // tell dtc to inspect the other pipe
                        // tell kcl to resume spiral
                        currentPhase = PipelinePipeInspectionPhase::LOOKING_FOR_PIPE;
                    } else {
                        // FAIL
                    }
                }
                return fsm::ok;
            }

            // tell kcl to move to the pipeline structure
            ctrlData->perceptionData.currentPipeDtc.vertical_distance; // has to be like 0.3m
        } else if (currentPhase == PipelinePipeInspectionPhase::ADDITIONAL_INSPECTION_LAP) {
            auto& pipe = (currentPipeDtcCode == "A") ? pipeA : pipeB;

            ctb::LatLong targetPosition;
            double distance, azimuthRad;
            if (pipe.movingToStructure) {
                // Check if arrived to start
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pipe.startPosition, distance, azimuthRad);
                if (distance < 0.5) {
                    pipe.numberOfLaps++;
                } else {
                    targetPosition = pipe.startPosition;
                    // Tell kcl to go to targetPosition
                }
            } else {
                // Check if arrived to end
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pipe.endPosition, distance, azimuthRad);
                if (distance < 0.5) {
                    pipe.numberOfLaps++;
                    pipe.movingToStructure = true;
                }else{
                    targetPosition = pipe.endPosition;
                    // Tell kcl to go to targetPosition
                }
            }
            if (pipe.numberOfLaps >= maxInspectionlaps) {
                onPipeInspection = false;
                auto& otherPipe = (currentPipeDtcCode == "A") ? pipeB : pipeA;
                if (otherPipe.numberOfLaps < maxInspectionlaps) {
                    // tell dtc to inspect the other pipe
                    // tell kcl to resume spiral
                    currentPhase = PipelinePipeInspectionPhase::LOOKING_FOR_PIPE;
                } else {
                    // FAIL
                }
            }
        }

        return fsm::ok;
    }

    fsm::retval StateInspectPipes::OnExit()
    {

        return fsm::ok;
    }
}
}
