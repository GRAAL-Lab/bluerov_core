#ifndef MARINE_DETECTOR_ROS2_H
#define MARINE_DETECTOR_ROS2_H


#include <utilities_ros2.h>
#include <unordered_map>
#include <memory>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

#include <marine_detector.h>

struct sensorTopics {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_cloudPub;  // sensor-related point cloud (e.g. lidar slice or pyramid-based cluster set)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_cloud2DPub;  // sensor-related point cloud (e.g. lidar slice or pyramid-based cluster set) in 2D
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_hullsPub;  // obstacle hulls
    rclcpp::Publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>::SharedPtr worldF_obstacleBoundingBoxesPub;  // obstacle bounding boxes
};

class MarineDetectorROS2 : public MarineDetector,  public rclcpp::Node {
public:
    /**
     * @brief Constructor for MarineDetectorROS2.
     * @param node Shared pointer to the ROS2 node.
     * @param bagPath Path to the data bag file.
     * @param isSim Flag to indicate if the detector is running in simulation.
     */
    MarineDetectorROS2(
        const std::string& bagPath, 
        const bool isSim
    );

private:
    void Init(const std::string& bagPath, bool isSim);
    void ObstacleDetectionCallbackRAMI(
        const obstacle_tracking_msg::msg::BoundingBox2DArray::ConstSharedPtr& ann_msg
    );

    rclcpp::Subscription<obstacle_tracking_msg::msg::BoundingBox2DArray>::SharedPtr cameraObstaclesSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr geoPoseStampedSub_;

    void ObstacleDetectionCallbackInternal(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg, const bool worldF_pos_set, const bool worldF_orient_set);

    void InitSubscribers() override;
    void InitPublishers() override;
    void Run() override;

    odtc::LimitedQueue<std::pair<rclcpp::Time, Eigen::TransformationMatrix>> worldF_poses_queue_;

    // Settings
    void FillSettingsMsg() override;
    obstacle_tracking_msg::msg::ObstDetectionSettings settingsMsg_;

    rclcpp::TimerBase::SharedPtr runTimer_;  ///< Timer for periodic execution.
    rclcpp::CallbackGroup::SharedPtr callback_group_;  ///< Callback group.
    rclcpp::executors::SingleThreadedExecutor executor_;  ///< Executor for spinning.

    using DetectionSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu>;
    using DetectionSyncPolicyStonefish = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, geographic_msgs::msg::GeoPoseStamped>;
    std::shared_ptr<message_filters::Synchronizer<DetectionSyncPolicy>> _sync;  ///< Synchronizer for sensor messages.
    std::shared_ptr<message_filters::Synchronizer<DetectionSyncPolicyStonefish>> _syncStonefish;  ///< Synchronizer for sensor messages.

    double spinRate_ = 10.0; ///< Spin rate in Hz.
    bool enableBasicPrints_ = true; ///< Enable or disable logging for basic prints.

    odtc::FrameType frameType_;

    bool SetWorldF_VehiclePosition() override;
    bool SetWorldF_VehicleOrientation() override;
    bool SetWorldF_VehicleTwist() override;
    bool SetPrevTrackInfo() override;
    bool SetFGRegions() override;
    bool SetAnnotations(std::string camId) override;
    bool SetWorldF_VehicleOrientationIMU(const sensor_msgs::msg::Imu::ConstSharedPtr& imuMsg);
    bool SetWorldF_VehiclePose_GeoPoseStamped(const nav_msgs::msg::Odometry::ConstSharedPtr& geoPoseStamped_msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPclSubDebug_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubDebug_;
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const { std::cerr << "[] Cloud received."; }
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) const { std::cerr << "[] IMU received."; }

    // Subscribers
    std::map<std::string, std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>> lidarCloudSub_;
    //std::shared_ptr<message_filters::Subscriber<geographic_msgs::msg::GeoPoseStamped>> geoPoseStampedSub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imuSub_;
    std::map<std::string, std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> imgSub_;
    std::map<std::string, std::shared_ptr<message_filters::Subscriber<obstacle_tracking_msg::msg::BoundingBox2DArray>>> imgAnnSub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> twistSub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> gnssSub_;
    std::shared_ptr<message_filters::Subscriber<obstacle_tracking_msg::msg::ObstacleArray>> tracksSub_;
    std::shared_ptr<message_filters::Subscriber<obstacle_tracking_msg::msg::BoundingBox2DArray>> fgRegionSub_;
    message_filters::Cache<obstacle_tracking_msg::msg::BoundingBox2DArray> cacheFgRegion_;

    // Caches
    std::map<std::string, std::shared_ptr<message_filters::Cache<sensor_msgs::msg::Image>>> cacheImg_;
    std::map<std::string, std::shared_ptr<message_filters::Cache<obstacle_tracking_msg::msg::BoundingBox2DArray>>> cacheAnnotations_;
    message_filters::Cache<geometry_msgs::msg::TwistStamped> cacheVehicleTwist_;
    message_filters::Cache<sensor_msgs::msg::NavSatFix> cacheGNSS_;
    message_filters::Cache<obstacle_tracking_msg::msg::ObstacleArray> cacheTracks_;

    // Publishers
    rclcpp::Publisher<obstacle_tracking_msg::msg::SliceAngles>::SharedPtr sensorFovPub_;
    rclcpp::Publisher<obstacle_tracking_msg::msg::ObstDetectionSettings>::SharedPtr settingsPub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuDataPub_;
    rclcpp::Publisher<obstacle_tracking_msg::msg::SliceAngles>::SharedPtr noCamSliceAnglesPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_cumulCloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_cumulCloud2DPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr worldF_vehiclePosePub_;
    rclcpp::Publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>::SharedPtr worldF_predictedTracksPub_;
    rclcpp::Publisher<obstacle_tracking_msg::msg::ObstDetectionStats>::SharedPtr statsPub_;
    rclcpp::Publisher<obstacle_tracking_msg::msg::PyramidArray>::SharedPtr pyramidsPub_;
    rclcpp::Publisher<obstacle_tracking_msg::msg::PyramidArray>::SharedPtr unfilteredPyramidsPub_;
    rclcpp::Publisher<obstacle_tracking_msg::msg::ObstacleArray>::SharedPtr obstaclesPub_;
    rclcpp::Publisher< obstacle_tracking_msg::msg::ObstacleArrayMap>::SharedPtr imgFiltersPub_;
    std::map<std::string, sensorTopics> sensorsPub_;

    rclcpp::Time tsRos_;
    bool enableRosDebugPrints_ = false;
};

#endif // MARINE_DETECTOR_ROS2_H
