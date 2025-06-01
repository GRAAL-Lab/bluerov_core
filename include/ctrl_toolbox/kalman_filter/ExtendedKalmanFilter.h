#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER__H_

#include "MeasurementKalmanFilter.h"
#include "ModelKalmanFilter.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <rml/RML.h>
#include <vector>

namespace ctb {

class ExtendedKalmanFilter {
public:
    // F, note that f must depend on the state and the input and the dimension of the state, the indexAngles indicates which
    // entries of the state are angles to be wrapped;
    ExtendedKalmanFilter(int stateDimension, std::vector<int> indexAngles, std::shared_ptr<ModelKalmanFilter> kalmanFilterModel);

    // Method that postpone to H the matrix output of the function, note that h is function of the stateitself
    void AddMeasurement(std::shared_ptr<MeasurementKalmanFilter> measurement);

    void Prediction(const Eigen::VectorXd& u);

    void Update(const Eigen::VectorXd& u);

    void Reset();

    void Init(const Eigen::VectorXd initialState, const Eigen::MatrixXd P);

    void Init(const Eigen::VectorXd initialState);

    auto StateVector() const -> const Eigen::VectorXd& { return x_; }

    auto PropagationError() const -> const Eigen::MatrixXd& { return P_; }

private:
    //state
    Eigen::VectorXd x_; // state
    Eigen::MatrixXd F_; //state transition Jacobian
    Eigen::MatrixXd R_; // observation(or measure) covariance (gaussian noise with zero mean)

    //measure
    Eigen::VectorXd z_; // measurements
    Eigen::MatrixXd H_; // observation matrix

    //Kalman notation
    Eigen::MatrixXd K_; // kalman gain

    Eigen::VectorXd predicted_z_; //observation prediction
    Eigen::MatrixXd S_; // Innovation (or residual) covariance

    Eigen::MatrixXd P_; // Predicted covariance estimate

    std::chrono::time_point<std::chrono::milliseconds> lastEstimationTime_;
    double speed_;
    int stateDimension_;
    std::shared_ptr<ModelKalmanFilter> kalmanFilterModel_;
    std::vector<int> indexAngles_;

    bool isFirst_;
    rml::RegularizationData regularizationParameter_;

    std::vector<std::shared_ptr<MeasurementKalmanFilter>> measurements;
};
}
#endif
