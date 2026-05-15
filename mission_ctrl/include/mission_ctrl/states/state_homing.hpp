#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateHoming : public StateBase {

        bool reachedSurface = false;

    public:
        StateHoming();
        ~StateHoming() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        ctb::LatLong homePosition;
        bool homePositionSet = false;

    };

}
}