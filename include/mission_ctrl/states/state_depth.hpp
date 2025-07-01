#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateDepth : public StateBase {

        bool doneInit = false;

    public:
        StateDepth();
        ~StateDepth() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        Eigen::Vector6d goalPose;

    };

}
}