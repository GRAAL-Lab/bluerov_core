#ifndef BRIDGE_MODE_HPP
#define BRIDGE_MODE_HPP

#include <string>

namespace auv_core_helper {

    namespace BrigdeMode {
        const std::string PoseCtrl = "PoseCtrl";
        const std::string VelCtrl = "VelCtrl";
    }

    namespace FlightMode {
        const std::string MANUAL = "MANUAL";
        const std::string STABILIZE = "STABILIZE";
        const std::string ALT_HOLD = "ALT_HOLD";
        const std::string GUIDED = "GUIDED";
        const std::string POSHOLD = "POSHOLD";
        const std::string SURFACE = "SURFACE";
        const std::string SURFTRAK = "SURFTRAK";
    }

}

#endif