#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateLatLong : public StateBase {

        double minAcceptanceRadius;

    public:
        StateLatLong();
        ~StateLatLong() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        ctb::LatLong goalPosition;
    };

}
}