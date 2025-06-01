#ifndef CTRL_TOOLBOX_HELPERFUNCTIONS_H
#define CTRL_TOOLBOX_HELPERFUNCTIONS_H

#include "GeographicLib/Geodesic.hpp"
#include "ctrl_toolbox/DataStructs.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>
#include <exception>
#include <libconfig.h++>
#include <ctrl_toolbox_internal/Futils.h>

namespace ctb {

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a pram from ConfigFIle
 */
template <typename A>
bool GetParam(const libconfig::Setting& confObj, A& param, const std::string& name) noexcept(false)
{
    if (!confObj.lookupValue(name, param)){
        std::cerr << tc::redL << "ctb::GetParam() Error: <" << name << "> lookup failed." << tc::none << std::endl;
        return false;
    }


    return true;
}

template <typename A>
bool GetParam(const libconfig::Config& confObj, A& param, const std::string& name) noexcept(false)
{
    if (!confObj.lookupValue(name, param)){
        std::cerr << tc::redL << "ctb::GetParam() Error: <" << name << "> lookup failed." << tc::none << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a vector pram from ConfigFIle
 */
template <typename A>
bool GetParamVector(const libconfig::Setting& confObj, A& param, const std::string& name) noexcept(false)
{
    try {
        const libconfig::Setting& settings = confObj.lookup(name);
        param.resize(settings.getLength());
        for (int n = 0; n < settings.getLength(); n++) {

            param(n) = settings[n];
        }
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "ctb::GetParamVector() Error: <" << name << "> lookup failed." << std::endl;
        return false;
    }


    return true;
}

template <typename A>
bool GetParamVector(const libconfig::Config& confObj, A& param, const std::string& name) noexcept(false)
{
    try {
        const libconfig::Setting& settings = confObj.lookup(name);
        param.resize(settings.getLength());
        for (int n = 0; n < settings.getLength(); n++) {

            param(n) = settings[n];
        }
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "ctb::GetParamVector() Error: <" << name << "> lookup failed." << std::endl;
        return false;
    }


    return true;
}

/**
 * @brief Helper clip function (available in std:: from c++17)
 */
double clamp(double n, double lower, double upper);

/**
 * @brief AngleDifference
 * @param from
 * @param to
 * @return
 */
double AngleDifference(double from, double to);

/**
 * @brief DistanceAndAzimuthRad
 * @param from
 * @param to
 * @param distance
 * @param azimuthrad
 */
void DistanceAndAzimuthRad(const LatLong& from, const LatLong& to, double& distance, double& azimuthrad);

/**
 * @brief Difference functor
 *
 * An utility templated functor to compute the difference between two objects
 * of the same type.
 */
template <typename T>
struct DifferenceFunctor {
    T operator()(T a, T b) const
    {
        return (a - b);
    }
};

struct AngleDifferenceFunctor {
    double operator()(double from, double to) const
    {
        return AngleDifference(from, to);
    }
};

/**
 * @brief Degree to radiant conversion function
 *
 */
double Deg2Rad(double deg);

/**
 * @brief Radiant to degree conversion function
 *
 */
double Rad2Deg(double rad);

template <class A>
void LocalUTM2LatLong(const A& cartesianPoint, const LatLong& centroid, LatLong& mapPoint, double& altitude)
{
    try {
        const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

        GeographicLib::LocalCartesian proj(centroid.latitude, centroid.longitude, 0, earth);
        {
            proj.Reverse(cartesianPoint[0], cartesianPoint[1], cartesianPoint[2], mapPoint.latitude, mapPoint.longitude, altitude);
        }
    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}

template <class A>
void LatLong2LocalUTM(const LatLong& mapPoint, double altitude, const LatLong& centroid, A& cartesianPoint)
{
    try {
        const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

        GeographicLib::LocalCartesian proj(centroid.latitude, centroid.longitude, 0, earth);
        {
            proj.Forward(mapPoint.latitude, mapPoint.longitude, altitude, cartesianPoint[0], cartesianPoint[1], cartesianPoint[2]);
        }
    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}

template <class A>
void LocalNED2LatLong(const A& cartesianPoint, const LatLong& centroid, LatLong& mapPoint, double& altitude)
{
    try {
        const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

        GeographicLib::LocalCartesian proj(centroid.latitude, centroid.longitude, 0, earth);
        {
            proj.Reverse(cartesianPoint[1], cartesianPoint[0], -cartesianPoint[2], mapPoint.latitude, mapPoint.longitude, altitude);
        }
    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}

template <class A>
void LatLong2LocalNED(const LatLong& mapPoint, double altitude, const LatLong& centroid, A& cartesianPoint)
{
    try {
        const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

        GeographicLib::LocalCartesian proj(centroid.latitude, centroid.longitude, 0, earth);
        {
            proj.Forward(mapPoint.latitude, mapPoint.longitude, altitude, cartesianPoint[1], cartesianPoint[0], cartesianPoint[2]);
            cartesianPoint[2] = -cartesianPoint[2];
        }
    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}

void NormalizeAngle(double& angle);

Eigen::VectorXd FilterAngularJump(const Eigen::VectorXd primaryHeading, const Eigen::VectorXd otherHeading);
}

#endif // CTRL_TOOLBOX_HELPERFUNCTIONS_H
