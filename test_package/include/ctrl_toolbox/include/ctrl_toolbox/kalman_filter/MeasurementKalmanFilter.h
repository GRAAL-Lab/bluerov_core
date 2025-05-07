#ifndef MEASURMENTKALMANFILTER_H
#define MEASURMENTKALMANFILTER_H

#include <eigen3/Eigen/Dense>

namespace ctb {
class MeasurementKalmanFilter {
public:
    MeasurementKalmanFilter(bool isAngleMeasure);

    virtual ~MeasurementKalmanFilter();

    virtual Eigen::MatrixXd ComputeJacobian(const Eigen::VectorXd& state) = 0; // H = der(h(x,u))/der(x)

    auto MeasureVector() const -> const Eigen::VectorXd& { return z_; }
    auto MeasureVector() -> Eigen::VectorXd& { return z_; }

    virtual Eigen::VectorXd ComputePrediction(const Eigen::VectorXd& state) = 0;

    auto Covariance() const -> const Eigen::MatrixXd& { return covariance_; }
    auto Covariance() -> Eigen::MatrixXd& { return covariance_; }

    auto IsAngleMeasure() const -> bool { return isAngleMeasure_; }

protected:
    Eigen::VectorXd z_;
    Eigen::MatrixXd covariance_;
    bool isAngleMeasure_;
};
}
#endif // MEASURMENTKALMANFILTER_H
