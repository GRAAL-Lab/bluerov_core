#include "mission_ctrl/states/state_base.hpp"
#include <cmath>

namespace mission {

namespace states {

    class StateSearchObject : public StateBase {
    public:
        StateSearchObject();
        ~StateSearchObject() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        fsm::retval StartSearch();

        //things to make it do the serpentine instead of the spiral
        bool reachedLeftmostPoint = false;
        bool sentPathFollowingCommand = false;
        std::queue<ctb::LatLong> areaPoints;
        
    };

}
}