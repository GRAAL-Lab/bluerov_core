#ifndef MODEL_KALMAN_FILTER_H_
#define MODEL_KALMAN_FILTER_H_

#include <chrono>
#include <eigen3/Eigen/Dense>
namespace ctb {
class ModelKalmanFilter {
public:
    ModelKalmanFilter();
    virtual ~ModelKalmanFilter();

    virtual Eigen::VectorXd ComputeStateTransitionModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) = 0; //f(x,u)
    virtual Eigen::MatrixXd ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input) = 0; //F = der(f(x,u)/der(x)

    auto Covariance() const -> const Eigen::MatrixXd& { return covariance_; }
    auto Covariance() -> Eigen::MatrixXd& { return covariance_; }

protected:
    std::chrono::system_clock::time_point last_comp_time_;
    Eigen::MatrixXd covariance_;
};
}

#endif
