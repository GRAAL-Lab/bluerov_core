/*
 * VirtualFrame.h
 *
 *  Created on: Apr 26, 2018
 *      Author: wanderfra
 */

#ifndef VIRTUAL_FRAME_H_
#define VIRTUAL_FRAME_H_

#include <ctrl_toolbox/HelperFunctions.h>
#include <libconfig.h++>
#include <rml/RML.h>
namespace ctb {

/*
 * @brief A simple virtual frame for smooth control
 *
 * The virtual frame is a simple concept: instead of feeding a new goal position <g> directly to the control frame <c> of
 * the robot a virtual frame is used to avoid discontinuities. The virtual frame is set initially coincident with the frame
 * <c>, then an integration process brings the virtual frame towards the goal frame. This virtual frame is then fed as "goal" to
 * the  frame which does not see any jumps in the goal position because they are filtered by the virtual frame.
 * A mechanism is implemented to avoid that the virtual frame "runs away" too much from the  frame.
 */
class VirtualFrame {

public:
    enum VFType {
        FullPose,
        Angular,
        Linear
    };

    /*
    * @brief Default constructor
    */
    VirtualFrame();
    /*
     * @brief Default desconstructor
     */
    ~VirtualFrame();
    /*
     * @brief Reset the virtual frame
     * Simply sets the virtual frame to the identity matrix and the error to zero
     */
    void ResetState();
    /*
     * @brief Reset the virtual frame to the given value
     * Sets the virtual frame to be the same as the given value
     * @param[in] worldF_T_virtualF the transformation matrix of the virtual frame w.r.t. the world frame
     */
    void ResetState(const Eigen::TransformationMatrix worldF_T_virtualF);
    /*
     * @brief Compute the new virtual frame position
     * The method updates the position of the virtual frame \<v\> on the basis of the goal frame <g>
     * The current worldF_T_startF is used to prevent the virtual frame <v> from getting too far away from <e>
     * @param[in] worldF_T_startF the frame of what we want to reach the goal
     * @param[in] worldF_T_goalF the current goal frame position
     * @param[out] worldF_T_virtualF the new virtual frame position
     */
    void Compute(const Eigen::TransformationMatrix& worldF_T_startF, const Eigen::TransformationMatrix &worldF_T_goalF, Eigen::TransformationMatrix& worldF_T_virtualF);
    /*
     * @brief Compute the new virtual frame position
     * The method updates the position of the virtual frame <v> on the basis of the requested Cartesian velocity xdotbar
     * The current  position worldF_T_startF is used to prevent the virtual frame <v> from getting too far away from <s>
     * @param[in] worldF_T_startF the current  frame position
     * @param[in] xdotbar the requested Cartesian velocity
     * @param[out] world_T_virtual the new virtual frame position
     */
    void Compute(const Eigen::TransformationMatrix& worldF_T_controlF, const Eigen::Vector6d& xdotbar, Eigen::TransformationMatrix& world_T_virtual);
    /*
     * @brief Method getting the error between the goal and the virtual frame
     */
    auto VirtualFrameToGoalError() const -> const Eigen::Vector6d& { return virtualFrameToGoalFrameError_; }
    /*
     * @brief Method getting the error between the virtual frame and the <e>
     */
    auto ControlFrameToVirtualFrameError() const -> const Eigen::Vector6d& { return controlFrameToVirtualFrameError_; }
    /**
     * @brief Method setting the projector rotation matrix, the normal to the plane must coincide with the z axis, the
     * transformation
     * matrix must be expressed wrt to the inertial frame.
     * @param worldF_R_projectionF rotation matrix in between the world and the projector frame
     */
    auto ProjectorFrame() -> Eigen::RotationMatrix& { return worldF_R_projectionF_; }
    /*
     * @brief Method getting the track error
     */
    auto TrackError() -> Eigen::Vector3d& { return errorTrack_; }
    /*
     * @brief Method getting the cross track error
     */
    auto CrossTrackError() -> Eigen::Vector3d& { return errorCrossTrack_; }
    /*
     * Struc with all the param that needs to use VirtualFrame class
     */
    struct VirtualFrameParams {
        double sampleTime;
        Eigen::Vector2d gain;
        Eigen::Vector2d onTrackAllowedDistance;
        Eigen::Vector2d crossTrackAllowedDistance;
        VFType vfType;

        bool ConfigureFromFile(const libconfig::Config& confObj, const std::string& taskName)
        {
            const libconfig::Setting& root = confObj.getRoot();
            const libconfig::Setting& states = root["tasks"];

            const libconfig::Setting& state = states.lookup(taskName);

            if (!ctb::GetParam(state, sampleTime, "virtualFrameSampleTime"))
                return false;
            if (!ctb::GetParamVector(state, gain, "virtualFrameGain"))
                return false;
            if (!ctb::GetParamVector(state, onTrackAllowedDistance, "onTrackAllowedDistance"))
                return false;
            if (!ctb::GetParamVector(state, crossTrackAllowedDistance, "crossTrackAllowedDistance"))
                return false;
            int tmp;
            if (!ctb::GetParam(state, tmp, "virtualFrameType"))
                return false;

            vfType = static_cast<VFType>(tmp);

            return true;
        }
    } virtualFrameParams;

    bool projectedOnPlane;

private:
    Eigen::Vector6d controlFrameToVirtualFrameError_;
    Eigen::Vector6d virtualFrameToGoalFrameError_;
    Eigen::Vector6d virtualFrameVelocity_;
    Eigen::Vector3d errorTrack_;
    Eigen::Vector3d errorCrossTrack_;
    Eigen::TransformationMatrix worldF_T_virtualF_;
    Eigen::TransformationMatrix worldF_T_goalF_;
    Eigen::TransformationMatrix worldF_T_virtualFInit_;
    Eigen::RotationMatrix worldF_R_projectionF_;
    Eigen::TransformationMatrix worldF_T_goalFCurrent_;
};
}

#endif /* VIRTUAL_FRAME_H_ */
