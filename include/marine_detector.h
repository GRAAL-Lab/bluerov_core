#ifndef MARINE_DETECTION_MAIN_H
#define MARINE_DETECTION_MAIN_H

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

//#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ctrl_toolbox/HelperFunctions.h>
#include <ctrl_toolbox_internal/Futils.h>

#include <odtc/ObstacleDetectionHandler.h>
#include <odtc/Camera.h>
#include <odtc/DataAssociation.h>

#include <offline_detection_utils.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudHost;

/**
 * \class MarineDetector
 * \brief Marime obstacle detector
 */
class MarineDetector {

    public:
    /**
     * \brief Constructor.
     * @param[in] dataPath: data path.
     * @param[in] isSim: true if is simulation, false otherwise.
     */
    MarineDetector(const std::string params, const bool isSim);
    MarineDetector() {}; /*!< Default constructor. */
    virtual ~MarineDetector() = 0; /*!< Default destructor. */
    
    auto IsSim() const -> bool { return this->isSim_; } /*!< Getter. */
    auto EnableDebugPrints() const -> bool { return this->enableDebugPrints_; } /*!< Getter. */
    auto EnableDebugPrints(bool enableDebugPrints) -> void { this->enableDebugPrints_ = enableDebugPrints; } /*!< Setter. */
    auto EnableBasicPrints() const -> bool { return this->enableBasicPrints_; } /*!< Getter. */
    auto EnableBasicPrints(bool enableBasicPrints) -> void { this->enableBasicPrints_ = enableBasicPrints; } /*!< Setter. */

    protected:
    /**
     * \brief Constructor body.
     * @param[in] dataPath: data path.
     * @param[in] isSim: true if is simulation, false otherwise.
     */ 
    void Init(const std::string dataPath, const bool isSim);
    void ReadConfigFile(const std::string& dataPath, libconfig::Config& confObj);
    odtc::DetectionSensorConfiguration LoadTestConfig(libconfig::Setting &testConfigSetting);

    bool isSim_ = true;
    bool enableDebugPrints_ = false;
    bool enableBasicPrints_ = true;
    bool enableRTAnnot_ = false;
    std::string datasetName_ = "";

    void ReadDetectionParams( libconfig::Setting &root, odtc::ObstacleDetectionParameters &odtc, odtc::Tracking &imgTracker);
    odtc::Camera CreateCamera(const libconfig::Setting& camSettings, const size_t camId, odtc::FrameType ft = odtc::FrameType::NED);
    odtc::Lidar CreateLidar(const libconfig::Setting& lidarSettings, const size_t lidarId, odtc::FrameType ft = odtc::FrameType::NED);

    virtual void FillSettingsMsg() = 0;
    virtual void InitSubscribers() = 0;
    virtual void InitPublishers() = 0;
    virtual void Run() = 0;

    void ConfigureClusteringStats(odtc::ClusteringStats& clusteringStatsLidar, std::vector<odtc::ClusteringStats>& clusteringStatsCam);

    std::vector<bool> rgbImgAvailable_;
    std::vector<odtc::BoundingBox<2>> fgRegions_;
    
    odtc::ObstacleDetectionHandler detection_;
    double spinRate_;
    bool enableStandaloneLidar_;
    bool enableCamera_;

    std::map<std::string, odtc::ClusteringStats> clusteringStats_;

    // Methods.
    void InitOpenCVWindows(const unsigned int nCams);
    //std::vector<odtc::BoundingBox<2>> GetBoxesFromROSMessage(darknet_ros_msgs::BoundingBoxesConstPtr boxesMsg);
    
    // Vehicle data
    Eigen::Vector6d worldF_vehicleTwist_;
    Eigen::TransformationMatrix worldF_T_vehicleF_;
    Eigen::TransformationMatrix worldF_T_vehicleF_t0_;
    Eigen::Vector3d llh_vehiclePos_;
    Eigen::Vector3d llh_vehiclePos_t0_;

    // Tracks
    std::vector<odtc::TrackInfo> prevWorldF_tracks_;

    // YOLO
    std::string imgDetectionModel_;
    std::string imgDetectionModel_ir_;

    // Point clouds
    odtc::PointCloudHandler<3> lidarF_newCloud_;

    // Sensor config
    odtc::DetectionSensorConfiguration dsc_;
    std::map<std::string, std::pair<double,double>> sensorName2FoV_;
    std::map<std::string, std::string> sensorName2AnnotationPath_;
    std::map<std::string, std::map<double, std::vector<odtc::BoundingBox<2>>>> imgOfflineAnnotations_;
    std::map<std::string, std::vector<odtc::BoundingBox<2>>> imgAnnotations_;

    // OpenCV windows.
    std::vector<std::string> opencvWindows_;

    // Initial conditions.
    double t0_ = 0.0;

    // Flow.
    double ts_ = -1;

    bool firstRun_;
    bool firstGNSSReceived_;

    double latestPrevTrackTs_ = -1;
    std::map<odtc::CameraId, double> cam2LatestAnnotationTs_;
    double latestFGRegionTs_ = -1;

    std::string mode_;

    bool saveClusters_;
    bool enableTrackingGt_;

};
#endif
