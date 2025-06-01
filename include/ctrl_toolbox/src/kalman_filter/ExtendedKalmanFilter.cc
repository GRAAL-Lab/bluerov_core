#include "kalman_filter/ExtendedKalmanFilter.h"
#include "HelperFunctions.h"
#include <rml/RML.h>

namespace ctb {

// F, note that f must depend on the state and the input and the dimension of the state, the indexAngles indicates
// which
// entries of the state are angles to be wrapped;
ExtendedKalmanFilter::ExtendedKalmanFilter(int stateDimension, std::vector<int> indexAngles, std::shared_ptr<ModelKalmanFilter> kalmanFilterModel)
    : stateDimension_ { stateDimension }
    , kalmanFilterModel_ { std::move(kalmanFilterModel) }
    , indexAngles_ { std::move(indexAngles) }
{
    x_.setZero(stateDimension_);
    P_.setZero(stateDimension_, stateDimension_);

    //regulatization parameters to perform speudoinverse
    regularizationParameter_.params.lambda = 0.000001;
    regularizationParameter_.params.threshold = 0.000001;

    isFirst_ = true;
}

// Method that postpone to H the matrix output of the function, note that h is function of the stateitself
void ExtendedKalmanFilter::AddMeasurement(std::shared_ptr<MeasurementKalmanFilter> measurement)
{
    measurements.push_back(measurement);
    isFirst_ = false;
}

void ExtendedKalmanFilter::Update(const Eigen::VectorXd& u)
{
    //Prediction
    F_ = kalmanFilterModel_->ComputeJacobian(x_, u); //The state transition Jacobian

    x_ = kalmanFilterModel_->ComputeStateTransitionModel(x_, u); //Predicted state estimate

    for (const auto i : indexAngles_) {
        NormalizeAngle(x_(i));
    }

    P_ = F_ * P_ * F_.transpose() + kalmanFilterModel_->Covariance(); //Predicted covariance estimate

    //Update
    if (!isFirst_) {

        //Get the first measure vector
        z_ = measurements.front()->MeasureVector();

        //Compute the first prediction measurement
        predicted_z_ = measurements.front()->ComputePrediction(x_);

        //Filter to avoid angular jump
        if (measurements.front()->IsAngleMeasure()) {
            predicted_z_ = FilterAngularJump(z_, predicted_z_);
        }

        //Get the jacobian of the first measurements
        H_ = measurements.front()->ComputeJacobian(x_);
        //Get the covariance of the first measurements
        R_ = measurements.front()->Covariance();

        //Compute the measure vectors, predictions, jacobians and covariances by iteratively stack vectors and matrices
        for (unsigned int i = 1; i < measurements.size(); i++) {

            Eigen::VectorXd z_tmp = measurements.at(i)->MeasureVector();
            z_ = rml::UnderJuxtapose(z_, z_tmp);

            Eigen::VectorXd predicted_z_tmp = measurements.at(i)->ComputePrediction(x_);
            if (measurements.at(i)->IsAngleMeasure()) {
                predicted_z_tmp = FilterAngularJump(z_tmp, predicted_z_tmp);
            }
            predicted_z_ = rml::UnderJuxtapose(predicted_z_, predicted_z_tmp);

            H_ = rml::UnderJuxtapose(H_, measurements.at(i)->ComputeJacobian(x_));

            Eigen::VectorXd R_diag = R_.diagonal();
            R_diag = rml::UnderJuxtapose(R_diag, measurements.at(i)->Covariance().diagonal());
            R_ = Eigen::MatrixXd::Zero(R_diag.size(), R_diag.size());
            R_.diagonal() = R_diag;
        }

        S_ = H_ * P_ * H_.transpose() + R_; //Innovation (or residual) covariance

        K_ = P_ * H_.transpose() * rml::RegularizedPseudoInverse(S_, regularizationParameter_); // Near-optimal Kalman gain

        x_ = x_ + K_ * (z_ - predicted_z_); // Updated state estimate

        for (const auto i : indexAngles_) {
            NormalizeAngle(x_(i));
        }

        P_ = P_ - K_ * H_ * P_; // Updated covariance estimate

        measurements.clear();
        isFirst_ = true;
    }
}

void ExtendedKalmanFilter::Reset()
{
    x_.setZero(stateDimension_);
    P_.setZero(stateDimension_, stateDimension_);
    measurements.clear();
    isFirst_ = true;
}

void ExtendedKalmanFilter::Init(const Eigen::VectorXd initialState, const Eigen::MatrixXd P)
{
    x_ = initialState;
    P_ = P;
    isFirst_ = true;
}

void ExtendedKalmanFilter::Init(const Eigen::VectorXd initialState)
{
    x_ = initialState;
    isFirst_ = true;
}
}
