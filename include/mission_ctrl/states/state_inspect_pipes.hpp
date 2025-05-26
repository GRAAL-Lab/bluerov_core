#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    enum PipelinePipeInspectionPhase {
        LOOKING_FOR_PIPE,
        MOVING_TO_POINT_ON_PIPE,
        MOVING_AWAY_FROM_PIPELINE_STRUCTURE,
        MOVING_TO_PIPELINE_STRUCTURE,
        ADDITIONAL_INSPECTION_LAP
    };

    struct PipelinePipeInspectionProgress{
        uint numberOfLaps = 0; // Number of laps over the pipe
        ctb::LatLong startPosition; // Where the pipe comes out of the structure
        ctb::LatLong endPosition; // Where the pipe ends
        bool movingToStructure = false; // Used in additional lap    
    };

    class StateInspectPipes : public StateBase {

        uint maxInspectionlaps = 3; // Maximum number of laps over one pipe

        std::string currentPipeDtcCode; // A or B
        PipelinePipeInspectionProgress pipeA;
        PipelinePipeInspectionProgress pipeB;

        bool onPipeInspection = false; // Currently inspecting a pipe, else looking for a pipe
        PipelinePipeInspectionPhase currentPhase; // Current phase of the pipe inspection
        
        bool missingRedMarker = false; // If the red marker was not found
        bool missingPipeNumber = false; // If the pipe number was not found

    public:
        StateInspectPipes();
        ~StateInspectPipes() override;

        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;


    };

}
}