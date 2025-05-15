#include "mission_ctrl/states/state_inspect_buoy.hpp"

namespace mission {

namespace states {

    StateInspectBuoy::StateInspectBuoy(){
    }

    StateInspectBuoy::~StateInspectBuoy() {
    }

    fsm::retval StateInspectBuoy::OnEntry(){       

        return fsm::ok;        
    }

    fsm::retval StateInspectBuoy::Execute(){
        
        return fsm::ok;       
    }

    fsm::retval StateInspectBuoy::OnExit(){
        
        return fsm::ok;
    }
}
}
