#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateLatLong : public StateBase {

        bool doneInit = false;
        double minAcceptanceRadius;
        double depthTolerance;

    public:
        StateLatLong();
        ~StateLatLong() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        Eigen::Vector6d goalPose;
    };

}
}