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

enum PerceptionState {
    IDLE, STANDARD, BUOYS, PIPES, MAIN_PIPE, ALL
};

struct sensorTopics {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_cloudPub;  // sensor-related point cloud (e.g. lidar slice or pyramid-based cluster set)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_cloud2DPub;  // sensor-related point cloud (e.g. lidar slice or pyramid-based cluster set) in 2D
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr worldF_hullsPub;  // obstacle hulls
    rclcpp::Publisher<image_pipeline_msgs::msg::BoundingBox2DArray>::SharedPtr worldF_obstacleBoundingBoxesPub;  // obstacle bounding boxes
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
    bool enableDbgPrint_ = false;
    double tLastDbgPrint_ = 0;

private:
    void Init(const std::string& bagPath, bool isSim);

    rclcpp::Subscription<image_pipeline_msgs::msg::BoundingBox2DArray>::SharedPtr cameraObstaclesSub_;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr geoPoseStampedSub_;

    void ObstacleDetectionCallbackInternal(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg, const bool worldF_pos_set, const bool worldF_orient_set);

    void InitSubscribers() override;
    void InitPublishers() override;
    void Run() override;

    odtc::LimitedQueue<std::pair<rclcpp::Time, Eigen::TransformationMatrix>> worldF_poses_queue_;

    // Settings
    void FillSettingsMsg() override;
    image_pipeline_msgs::msg::ObstDetectionSettings settingsMsg_;

    rclcpp::TimerBase::SharedPtr runTimer_;  ///< Timer for periodic execution.
    rclcpp::CallbackGroup::SharedPtr callback_group_;  ///< Callback group.
    rclcpp::executors::SingleThreadedExecutor executor_;  ///< Executor for spinning.
    
    double spinRate_ = 10.0; ///< Spin rate in Hz.
    bool enableBasicPrints_ = true; ///< Enable or disable logging for basic prints.

    odtc::FrameType frameType_;

    bool PerceptionCallback(const auv_core_helper::msg::PoseStamped::ConstSharedPtr& geoPoseStamped_msg);
    Pipe GetPipeInfo(const image_pipeline_msgs::msg::PipeDirection::ConstPtr msg);

    // Subscribers
    message_filters::Cache<image_pipeline_msgs::msg::PipeDirection> pipeCache_;
    message_filters::Cache<image_pipeline_msgs::msg::BoundingBox2DArray> imgAnnCache_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imuSub_;
    std::map<std::string, std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> imgSub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> gnssSub_;
    std::shared_ptr<message_filters::Subscriber<image_pipeline_msgs::msg::PipeDirection>> pipeSub_;
    std::shared_ptr<message_filters::Subscriber<image_pipeline_msgs::msg::BoundingBox2DArray>> imgAnnSub_;


    // Caches
    std::map<std::string, std::shared_ptr<message_filters::Cache<sensor_msgs::msg::Image>>> cacheImg_;
    message_filters::Cache<sensor_msgs::msg::NavSatFix> cacheGNSS_;

    // Publishers
    rclcpp::Publisher<image_pipeline_msgs::msg::ObstDetectionSettings>::SharedPtr settingsPub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuDataPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr worldF_vehiclePosePub_;
    rclcpp::Publisher<image_pipeline_msgs::msg::ObstDetectionStats>::SharedPtr statsPub_;
    rclcpp::Publisher<image_pipeline_msgs::msg::Obstacles>::SharedPtr obstaclesPub_;
    std::map<std::string, sensorTopics> sensorsPub_;

    rclcpp::Subscription<auv_core_helper::msg::MissionStatus>::SharedPtr missionStatusSub_;
    void MissionStatusCallback(const auv_core_helper::msg::MissionStatus::SharedPtr msg);

    DtcRequest currentRequest_;

    rclcpp::Time tsRos_;
    bool enableRosDebugPrints_ = false;

    PerceptionState state = PerceptionState::ALL;
};

#endif // MARINE_DETECTOR_ROS2_H
