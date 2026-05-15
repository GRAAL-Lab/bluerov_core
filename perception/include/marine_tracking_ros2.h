#ifndef MARINE_TRACKING_MAIN_ROS2_H
#define MARINE_TRACKING_MAIN_ROS2_H

#include <vector>
#include <chrono>
#include <string>
#include <experimental/filesystem>
#include <iostream>
#include <ctime>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <omp.h> // OPENMP

#include <utilities_ros2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ctrl_toolbox/HelperFunctions.h>
#include <ctrl_toolbox_internal/Futils.h>

#include <odtc/tracking.h>

/**
 * \class MarineDetector
 * \brief Marime obstacle detector
 */
class MarineTrackingROS2 : public rclcpp::Node {

    public:
    /**
     * \brief Constructor.
     * @param[in] dataPath: data path.
     * @param[in] isSim: true if is simulation, false otherwise.
     */
    MarineTrackingROS2(
        const std::string& bagPath, 
        const bool isSim);
    
    void Run();
    
    private:
    double ts_;
    double t0_ = -1;

    double trackingDt_ = 0.1;
    bool enableDbgPrint_ = false;
    double tLastDbgPrint_ = 0;

    bool firstRun_ = true;
    bool isFirstMsg_ = true;
    std::string datasetName_ = "";

    odtc::Tracking tracker_;
    void ReadTrackingParams(odtc::TrackingParams &trackingParams, std::map<std::string, odtc::IDAssocParams> &assocParams, libconfig::Setting &rootParams);
    
    rclcpp::TimerBase::SharedPtr runTimer_;  ///< Timer for periodic execution.
    rclcpp::CallbackGroup::SharedPtr callback_group_;  ///< Callback group.
    rclcpp::executors::SingleThreadedExecutor executor_;  ///< Executor for spinning.

    rclcpp::Time tsRos_;
    void FiltersCallback(const image_pipeline_msgs::msg::Obstacles::ConstPtr& obstaclesMsg);
    bool SetTime(const image_pipeline_msgs::msg::Obstacles::ConstPtr& obstacles);
    void ReadConfigFile(const std::string& dataPath, libconfig::Config& confObj);

    // Subscribers
    rclcpp::Publisher< auv_core_helper::msg::DtcList>::SharedPtr filtersPub_;

    message_filters::Cache<image_pipeline_msgs::msg::Obstacles> cacheDetections_;
    std::shared_ptr<message_filters::Subscriber<image_pipeline_msgs::msg::Obstacles>> detectionsSub_;

    rclcpp::Time tsROS_;
    Eigen::Vector3d llh_vehiclePos_t0_;

    bool enableDetectionRevision_ = true;
    std::map<std::string, std::string> goodLabelMappings;

    std::map<odtc::TrackId, odtc::RegistrationData> trackId2WorldFRegData_;
};
#endif
