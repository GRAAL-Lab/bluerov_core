#ifndef CTRL_TOOLBOX_DATASTRUCTS_H
#define CTRL_TOOLBOX_DATASTRUCTS_H

#include <rml/RML.h>

namespace ctb {

struct PIDGains {
    double Kp = { 0.0 }; //!< Proportional gain
    double Ki = { 0.0 }; //!< Integral gain
    double Kd = { 0.0 }; //!< Derivative gain
    double Kff = { 0.0 }; //!< Feed-forward gain
    double N = { 0.0 }; //!< Maximum gain for derivative part
    double Tr = { 0.0 }; //!< Tracking time constant for anti-windup

    PIDGains() = default;
};

struct LatLong {
    double latitude, longitude;
    LatLong()
        : latitude(0.0)
        , longitude(0.0)
    {
    }

    LatLong(double lat, double lon)
        : latitude(lat)
        , longitude(lon)
    {
    }

    friend std::ostream& operator<< (std::ostream& os, const LatLong& obj) {
        return os
            << "Lat, Long: (" << obj.latitude << ", " << obj.longitude << ")";
    };
};
}

#endif // CTRL_TOOLBOX_DATASTRUCTS_H
