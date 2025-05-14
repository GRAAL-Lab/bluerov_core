#include "state_inspect_buoy.hpp"

namespace mission {

namespace states {

    StateInspectBuoy:StateInspectBuoy(){
    }

    StateInspectBuoy::~StateInspectBuoy() {
    }

    fsm::retval StateInspectBuoy::OnEntry(){           
        return fsm::ok;        
    }

    fsm::retval StateInit::Execute(){
        
    }

    fsm::retval StateInit::OnExit(){
        return fsm::ok;
    }
}
}
