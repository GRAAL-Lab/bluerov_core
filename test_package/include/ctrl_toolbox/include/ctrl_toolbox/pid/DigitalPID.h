/*
 * DigitalPID.h
 *
 *  Created on: March 28, 2018
 *      Author: wanderfra
 */

#ifndef SRC_CTRL_DIGITALPID_H_
#define SRC_CTRL_DIGITALPID_H_

#include "ctrl_toolbox/DataStructs.h"
#include "ctrl_toolbox/HelperFunctions.h"
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <vector>

namespace ctb {

/**
 * @class DigitalPID
 *
 * @brief Impletation of a flexible 1-D digital PID controller
 *
 * This class implements a PID digital controller, with anti-windup and
 * feedforward control. The function to compute the error \f$ e \f$ can be configured
 * passing an std::function to SetErrorFunction(), function which has to take
 * two parameters \p reference and \p feedback and return the error.
 * The default error function is just the difference \f$ ref-fbk \f$, implemented
 * with the ctb::DifferenceFunctor. The output \f$ u \f$ is evaluated using the following
 * formula:\n
 *
 * \f$ u = K_p \cdot e + I + D + K_{ff} \cdot ref \f$
 *
 * where :\n
 *
 * \f$ I(t_k) = I(t_{k-1}) + (K_i \cdot T_s) \cdot e + \frac{T_s}{T_r} \cdot (u(t_{k}) - Sat(u(t_{k}))) \f$
 *
 * \f$ D(t_k) = \frac{T_d}{T_d + N \cdot T_s} \cdot D(t_{k-1}) - \frac{K_p \cdot T_d \cdot N}{T_d + N \cdot T_s} \cdot (y(t_k) - y(t_{k-1})) \f$
 *
 * The reference scientific paper for all the formulas derivation can be found at this link:
 * <a href="https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf" target="_blank">PID Control - Control and Dynamical Systems</a>
 *
 *
 */
class DigitalPID {
public:
    DigitalPID();

    DigitalPID(const PIDGains& gains, double sampleTime, double saturation);

    virtual ~DigitalPID();

    void Initialize(const PIDGains& gains, double sampleTime, double saturation);

    /**
	 * @brief Sets the control sampling time.
	 */
    void SetSampleTime(double Ts);

    /**
	 * @brief Sets the output saturation value.
	 */
    void SetSaturation(double uMax);

    /**
	 * @brief Resets the PID state.
	 */
    void Reset();

    /**
	 * @brief Main function: computes the PID output.
	 *
	 * @param ref	Current reference
	 * @param fbk	Current feedback
	 * @return	The PID controlled output
	 */
    double Compute(double GetRef, double GetFbk);

    void SetErrorFunction(const std::function<double(double, double)>& errorFunction);

    PIDGains GetGains() const;
    void SetGains(const PIDGains& gains);

    double GetRef() const;

    double GetFbk() const;

    double GetOutput() const;

private:
    PIDGains g_;
    double ref_;
    double Ts_; //!< Sample Time
    double uMax_; //!< Output saturation value
    std::vector<double> u_; //!< PID Output
    std::vector<double> e_; //!< Error
    std::vector<double> y_; //!< Feedback
    double D_; //!< Derivative Term
    double I_; //!< Integral Term
    std::function<double(double, double)> ErrorFunction_; //!< Definition of the error function that is used in Compute()
    bool PIDInitialized_;
    bool hasBeenReset_; //!< Utility variable to check if the PID has been Reset()
    bool TrToBeSetted_;
};
}

#endif /* SRC_CTRL_DIGITALPID_H_ */
