#include "mission_ctrl/states/state_base.hpp"
#include "sisl_toolbox/generic_curve.hpp"
#include "sisl_toolbox/path.hpp"

namespace mission {

namespace states {

    class StateCrossGate : public StateBase {
    public:
        StateCrossGate();
        ~StateCrossGate() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        bool reachedFrontOfGate = false; // If the vehicle reached the front of the gate
        ctb::LatLong firstWp, secondWp;

    };

}
}