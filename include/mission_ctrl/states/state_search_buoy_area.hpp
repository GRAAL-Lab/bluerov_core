#include "mission_ctrl/states/state_base.hpp"
#include "queue"

namespace mission {

namespace states {

    class StateSearchBuoyArea : public StateBase {

    public:
        StateSearchBuoyArea();
        ~StateSearchBuoyArea() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        uint numberOfBuoys;
        uint numberOfBuoysInspected;
        bool resumeSearch = false;

        bool reachedLeftmostPoint = false;
        bool sentPathFollowingCommand = false;
        std::queue<ctb::LatLong> areaPoints;

        int debugCounter = 0;

    };

}
}