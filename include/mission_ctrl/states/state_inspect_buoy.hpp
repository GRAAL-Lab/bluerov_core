#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateInspectBuoy : public StateBase {

    public:
        StateInspectBuoy();
        ~StateInspectBuoy() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        int debugCounter;
        bool reachedInspectionPosition = false; // Flag to check if the inspection position is reached
        ctb::LatLong inspectionPosition;
        Buoy buoyToInspect; // Store the buoy being inspected
        std::chrono::time_point<std::chrono::steady_clock> inspectionStartTime;

        bool sentDepthCmd = false;
        double goalDepth= 0.0;
        std::chrono::time_point<std::chrono::steady_clock> beforeChangingDepthTime;
        BuoyActionColorMap buoyActionColorMap;

        bool GetBuoyToInspect();

        
    };

}
}