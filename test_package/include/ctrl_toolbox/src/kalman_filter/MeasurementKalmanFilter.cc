#include "kalman_filter/MeasurementKalmanFilter.h"

namespace ctb {

MeasurementKalmanFilter::MeasurementKalmanFilter(bool isAngleMeasure)
    : isAngleMeasure_{ isAngleMeasure }
{
}

MeasurementKalmanFilter::~MeasurementKalmanFilter() {}
}
