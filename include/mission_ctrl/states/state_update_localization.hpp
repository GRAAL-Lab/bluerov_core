#include "mission_ctrl/states/state_base.hpp"

namespace mission {

namespace states {

    class StateUpdateLocalization : public StateBase {

        ctb::LatLong startingPosition;
        double maxTimeForLocalization = 5.0; // seconds
        
        rclcpp::Time localizationStartTime;
        bool onSurface = false;
        bool diving = false;
        
    public:
        StateUpdateLocalization();
        ~StateUpdateLocalization() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

    };

}
}