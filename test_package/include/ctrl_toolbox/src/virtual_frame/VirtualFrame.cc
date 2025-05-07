/*
 * ctrl_cartesian.cc
 *
 *  Created on: Jun 14, 2012
 *      Author: enrico
 */

#include "virtual_frame/VirtualFrame.h"

namespace ctb {

VirtualFrame::VirtualFrame()
    : projectedOnPlane{ false }
{
    virtualFrameParams.gain.setZero();
    virtualFrameParams.sampleTime = 0.0;
    virtualFrameParams.onTrackAllowedDistance = { 4.0, 4.5 };
    virtualFrameParams.crossTrackAllowedDistance = { 1.0, 1.5 };
    virtualFrameParams.vfType = VFType::FullPose;
}

VirtualFrame::~VirtualFrame() {}

void VirtualFrame::ResetState()
{
    worldF_T_virtualF_.setIdentity();
    worldF_T_virtualFInit_.setIdentity();
    controlFrameToVirtualFrameError_.setZero();
}

void VirtualFrame::ResetState(const Eigen::TransformationMatrix worldF_T_virtualF)
{
    worldF_T_virtualF_ = std::move(worldF_T_virtualF);
    worldF_T_virtualFInit_ = std::move(worldF_T_virtualF);
    controlFrameToVirtualFrameError_.setZero();
}

void VirtualFrame::Compute(const Eigen::TransformationMatrix& worldF_T_startF, const Eigen::TransformationMatrix& worldF_T_goalF, Eigen::TransformationMatrix& worldF_T_virtualF)
{
    worldF_T_goalF_ = worldF_T_goalF;

    if (virtualFrameParams.gain.norm() == 0.0 || virtualFrameParams.sampleTime == 0.0) {
        std::cerr << "WARNING: No virtualFrameGain and/or sampleTime set!!" << std::endl;
    }

    virtualFrameToGoalFrameError_ = rml::CartesianError(worldF_T_virtualF_, worldF_T_goalF_);

    if (projectedOnPlane) {
        Eigen::Vector3d worldF_planeDir = worldF_R_projectionF_.col(2);
        Eigen::Matrix3d P = (Eigen::Matrix3d::Identity() - worldF_planeDir * worldF_planeDir.transpose());
        virtualFrameToGoalFrameError_.LinearVector(P * virtualFrameToGoalFrameError_.LinearVector());
        virtualFrameToGoalFrameError_.AngularVector(P * virtualFrameToGoalFrameError_.AngularVector());
    }

    virtualFrameVelocity_ = { virtualFrameToGoalFrameError_.LinearVector() * virtualFrameParams.gain[0], virtualFrameToGoalFrameError_.AngularVector() * virtualFrameParams.gain[1] };

    Compute(worldF_T_startF, virtualFrameVelocity_, worldF_T_virtualF);
}

void VirtualFrame::Compute(const Eigen::TransformationMatrix& worldF_T_controlF, const Eigen::Vector6d& xdotbar, Eigen::TransformationMatrix& world_T_virtual)
{
    controlFrameToVirtualFrameError_ = rml::CartesianError(worldF_T_controlF, world_T_virtual);

    if (projectedOnPlane) {
        Eigen::Vector3d projectorVector_worldFrame = worldF_R_projectionF_.col(2);
        Eigen::Matrix3d P = (Eigen::Matrix3d::Identity() - projectorVector_worldFrame * projectorVector_worldFrame.transpose());
        controlFrameToVirtualFrameError_.LinearVector(P * controlFrameToVirtualFrameError_.LinearVector());
        controlFrameToVirtualFrameError_.AngularVector(P * controlFrameToVirtualFrameError_.AngularVector());
    }

    bool outOfReach = false;
    Eigen::Vector3d errorLinear, xdotbarLinear;

    if (virtualFrameParams.vfType != FullPose) {
        errorLinear = controlFrameToVirtualFrameError_.LinearVector();
        xdotbarLinear = xdotbar.LinearVector();
        Eigen::Vector3d n_vg = (worldF_T_goalF_.TranslationVector() - worldF_T_virtualF_.TranslationVector()).normalized();
        errorTrack_ = (n_vg * n_vg.transpose()) * errorLinear;
        errorCrossTrack_ = (Eigen::Matrix3d::Identity() - n_vg * n_vg.transpose()) * errorLinear;
        if ((errorLinear + xdotbarLinear * virtualFrameParams.sampleTime).norm() > errorLinear.norm()) {
            outOfReach = true;

            double sigma;
            double sigmaTrack = rml::DecreasingBellShapedFunction(virtualFrameParams.onTrackAllowedDistance(0), virtualFrameParams.onTrackAllowedDistance(1), 0, 1, errorTrack_.norm());
            double sigmaCross = rml::DecreasingBellShapedFunction(virtualFrameParams.crossTrackAllowedDistance(0), virtualFrameParams.crossTrackAllowedDistance(1), 0, 1, errorCrossTrack_.norm());
            sigma = std::min(sigmaTrack, sigmaCross);
            virtualFrameVelocity_.LinearVector(xdotbarLinear * sigma);
        }
    }
    if (!outOfReach) {
        virtualFrameVelocity_ = xdotbar;
    }
    // move the virtual frame
    worldF_T_virtualF_ = worldF_T_virtualF_.Integral(virtualFrameVelocity_, virtualFrameParams.sampleTime);
    worldF_T_goalFCurrent_ = worldF_T_virtualF_;
}
}
