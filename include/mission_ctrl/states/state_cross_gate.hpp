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

        std::shared_ptr<Gate> gate;

        std::shared_ptr<sisl::Path> path;

        fsm::retval genPath();


    };

}
}