#ifndef UTILITIES_ROS2_H
#define UTILITIES_ROS2_H

#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.h>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <image_pipeline_msgs/msg/pipe_direction.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <detav_msgs/msg/obstacle.hpp>
#include <detav_msgs/msg/obstacle_list.hpp>
#include <detav_msgs/msg/size.hpp>
#include <detav_msgs/msg/size_with_covariance.hpp>

#include <obstacle_tracking_msg/msg/buoy.hpp>
#include <obstacle_tracking_msg/msg/marker.hpp>
#include <obstacle_tracking_msg/msg/pipe.hpp>
#include <obstacle_tracking_msg/msg/number.hpp>
#include <obstacle_tracking_msg/msg/obstacles.hpp>
#include <obstacle_tracking_msg/msg/bounding_box2_d.hpp>
#include <obstacle_tracking_msg/msg/bounding_box2_d_array.hpp>
#include <obstacle_tracking_msg/msg/obstacle.hpp>
#include <obstacle_tracking_msg/msg/obstacle_array.hpp>
#include <obstacle_tracking_msg/msg/obst_detection_settings.hpp>
#include <obstacle_tracking_msg/msg/obst_detection_stats.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <odtc/Utilities.h>
#include <odtc/ObstacleDetectionHandler.h>
#include <odtc/tracking.h>
#include <ctrl_toolbox/HelperFunctions.h>
#include <ctrl_toolbox_internal/Futils.h>


enum class TrackType { ENU, IMG };

class UtilitiesROS2 {

  public:

    // Time
    static double ROSTimeToTimestamp(const rclcpp::Time stamp); /*!< ROS time conversion. */
    static rclcpp::Time TimestampToROSTime(const double timestamp); /*!< Inverse ROS time conversion. */
    static bool TimestampsAreClose(const rclcpp::Time stamp1, const rclcpp::Time stamp2, const double tol = 0); /*!< ROS timestamp closeness. */

    // Poses and velocities
    static Eigen::TransformationMatrix ROSPoseToTransformMatrix(const geometry_msgs::msg::Pose& msg);
    static Eigen::TransformationMatrix ROSPoseToTFMatrix(const geometry_msgs::msg::PoseStamped::ConstPtr& imuMsg); /*!< ROS stamped \pose conversion. */
    static Eigen::RotationMatrix ROSImuMsgToRotation(const sensor_msgs::msg::Imu::ConstPtr& imuMsg); /*!< ROS IMU msg angular part conversion. */
    static Eigen::Vector6d ROSTwistToTwist(const geometry_msgs::msg::TwistStamped::ConstPtr twistMsg); /*!< ROS stamped twist conversion. */
    
    // Convert ODTC to ROS
    static obstacle_tracking_msg::msg::BoundingBox2D GetBox2DMsg(const rclcpp::Time t, const odtc::BoundingBox<2> b);
    
    // Convert ROS to ODTC
    static odtc::BoundingBox<2> GetBox2DFromMsg(const obstacle_tracking_msg::msg::BoundingBox2D &boxMsg);
    
    // Publishing
    static void PublishBoundingBoxes2D(const rclcpp::Publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>::SharedPtr pub,
                                            const rclcpp::Time &t, const std::vector<odtc::BoundingBox<2>> &boxes);
    static void PublishPose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub, const rclcpp::Time t,
                                            const Eigen::TransformationMatrix T, std::string frames);
    static void PublishStats(const rclcpp::Publisher<obstacle_tracking_msg::msg::ObstDetectionStats>::SharedPtr pub, const rclcpp::Time t,
                                            std::map<std::string, odtc::ClusteringStats> &stats);

    // Read from cache
    static bool ReadImageFromCache(const message_filters::Cache<sensor_msgs::msg::Image> &imgCache, const rclcpp::Time stamp, sensor_msgs::msg::Image::ConstPtr &imageMsgPtr, const double maxTimeLag_s);

   
    static obstacle_tracking_msg::msg::Obstacle FillObstacleMsg(rclcpp::Time t, std::shared_ptr<odtc::BoundingBox<2>> box,
              std::shared_ptr<odtc::TrackData> filterInfo, const std::map<std::string, odtc::IDAssocParams> &assocParams);
    static obstacle_tracking_msg::msg::ObstacleArray FillObstacleArrayMsg(rclcpp::Time t, const odtc::Tracking &obstacles, const TrackType trackType, const Eigen::Vector3d &llhCentroid = Eigen::Vector3d(0,0,0),
        std::vector<odtc::BoundingBox<2>> boxes = {});
    static obstacle_tracking_msg::msg::ObstacleArray FillObstacleArrayMsg(rclcpp::Time t, std::vector<odtc::Obstacle<2>> &obstacles,
      std::vector<odtc::PolarRegion> &excludedRegions, const Eigen::Vector3d &llhCentroid, const Eigen::TransformationMatrix &worldF_T_vehicleF,
      std::map<size_t,std::vector<odtc::DetectionInfo>> di = {});
    static std::vector<odtc::Obstacle<2>> GetObstaclesFromROSMsg(const obstacle_tracking_msg::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid);
    static std::vector<odtc::Obstacle<2>> GetObstaclesFromROSMsg(const obstacle_tracking_msg::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid, std::map<odtc::TrackId, odtc::RegistrationData>& trackId2WorldFRegData_);
    static std::vector<odtc::Obstacle<2>> GetTracksFromROSMsg(const obstacle_tracking_msg::msg::ObstacleArray &msg);
    static detav_msgs::msg::ObstacleList FillDetavObstacleArrayMsg(rclcpp::Time t, const odtc::Tracking &trck, const Eigen::Vector3d &llhCentroid);
    static detav_msgs::msg::Obstacle FillDetavObstacleMsg(const odtc::TrackData &tr, const Eigen::Vector3d &llhCentroid);

    static bool ReadROSObstacleArray(const message_filters::Cache<obstacle_tracking_msg::msg::ObstacleArray> &cache, const rclcpp::Time t,
      obstacle_tracking_msg::msg::ObstacleArray::ConstPtr &obstacles, const double maxTimeLag_s);
};

#endif
