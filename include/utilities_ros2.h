#ifndef UTILITIES_RAMI_ROS2_H
#define UTILITIES_RAMI_ROS2_H

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

#include <image_pipeline_msgs/msg/buoy.hpp>
#include <image_pipeline_msgs/msg/marker.hpp>
#include <image_pipeline_msgs/msg/pipe.hpp>
#include <image_pipeline_msgs/msg/number.hpp>
#include <image_pipeline_msgs/msg/obstacles.hpp>
#include <image_pipeline_msgs/msg/bounding_box2_d.hpp>
#include <image_pipeline_msgs/msg/bounding_box2_d_array.hpp>
#include <image_pipeline_msgs/msg/obstacle.hpp>
#include <image_pipeline_msgs/msg/obstacle_array.hpp>
#include <image_pipeline_msgs/msg/obst_detection_settings.hpp>
#include <image_pipeline_msgs/msg/obst_detection_stats.hpp>
#include <auv_core_helper/msg/mission_status.hpp>
#include <auv_core_helper/msg/buoy.hpp>
#include <auv_core_helper/msg/generic_obstacle.hpp>
#include <auv_core_helper/msg/dtc_request.hpp>
#include <auv_core_helper/msg/dtc_list.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <odtc/Utilities.h>
#include <odtc/ObstacleDetectionHandler.h>
#include <odtc/tracking.h>
#include <ctrl_toolbox/HelperFunctions.h>
#include <ctrl_toolbox_internal/Futils.h>

namespace objectNames {
  const std::string BUOY_NAME = "Buoy";
  const std::string MAINPIPE_NAME = "MainPipe";
  const std::string PIPESTRUCT_NAME = "PipeStruct";
  const std::string MARKER_NAME = "Marker";
  const std::string NUMBER_NAME = "Number";
}

// Define the DtcRequest message
struct DtcRequest {
  bool obstacles;
  bool buoys;
};

// Define the MissionStatus message
struct MissionStatus {
  builtin_interfaces::msg::Time stamp;
  std::string task_benchmark;
  std::string state;
  std::string state_object;
  DtcRequest requests;
};

struct Buoy {
  size_t id;
  Eigen::TransformationMatrix wF_pose;
  std::string color;
  double radius;
  std::string notes;
};

struct Number {
  size_t id;
  Eigen::TransformationMatrix wF_pose;
  std::string bgColor;
  int number;
  std::string notes;
};

struct Marker {
  size_t id;
  Eigen::TransformationMatrix wF_pose;
  std::string color;
  std::string notes;
};

struct Pipe {
  double ts_;
  size_t id;
  Eigen::TransformationMatrix wF_pose;
  Eigen::TransformationMatrix wF_startPose;
  Eigen::TransformationMatrix wF_endPose;
  Eigen::Vector3d boxSize;
  std::vector<Number> numbers;
  std::vector<Marker> markers;
  std::string notes;
};

struct ObstaclesData {
  std::vector<Buoy> buoys;
  std::vector<Marker> markers;
  std::vector<Number> numbers;
  std::vector<Pipe> pipes;
  ctb::LatLong centroid;
};

enum class TrackType { ENU, IMG };

class UtilitiesROS2 {

  public:

    static double BuoyColorToDiameter(std::string);

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
    static image_pipeline_msgs::msg::BoundingBox2D GetBox2DMsg(const rclcpp::Time t, const odtc::BoundingBox<2> b);
    
    // Convert ROS to ODTC
    static odtc::BoundingBox<2> GetBox2DFromMsg(const image_pipeline_msgs::msg::BoundingBox2D &boxMsg);
    
    // Publishing
    static void PublishBoundingBoxes2D(const rclcpp::Publisher<image_pipeline_msgs::msg::BoundingBox2DArray>::SharedPtr pub,
                                            const rclcpp::Time &t, const std::vector<odtc::BoundingBox<2>> &boxes);
    static void PublishPose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub, const rclcpp::Time t,
                                            const Eigen::TransformationMatrix T, std::string frames);
    static void PublishStats(const rclcpp::Publisher<image_pipeline_msgs::msg::ObstDetectionStats>::SharedPtr pub, const rclcpp::Time t,
                                            std::map<std::string, odtc::ClusteringStats> &stats);

    // Read from cache
    static bool ReadImageFromCache(const message_filters::Cache<sensor_msgs::msg::Image> &imgCache, const rclcpp::Time stamp, sensor_msgs::msg::Image::ConstPtr &imageMsgPtr, const double maxTimeLag_s);

    static image_pipeline_msgs::msg::Obstacle FillObstacleMsg(rclcpp::Time t, std::shared_ptr<odtc::BoundingBox<2>> box,
              std::shared_ptr<odtc::TrackData> filterInfo, const std::map<std::string, odtc::IDAssocParams> &assocParams);
    static auv_core_helper::msg::DtcList FillObstacleArrayMsg(rclcpp::Time t, const odtc::Tracking &trck, const TrackType trackType,
              const Eigen::Vector3d &llhCentroid, std::vector<odtc::BoundingBox<2>> boxes = {});
    static image_pipeline_msgs::msg::ObstacleArray FillObstacleArrayMsg(rclcpp::Time t, std::vector<odtc::Obstacle<2>> &obstacles,
      std::vector<odtc::PolarRegion> &excludedRegions, const Eigen::Vector3d &llhCentroid, const Eigen::TransformationMatrix &worldF_T_vehicleF,
      std::map<size_t,std::vector<odtc::DetectionInfo>> di = {});
    static std::vector<odtc::Obstacle<2>> GetObstaclesFromROSMsg(const image_pipeline_msgs::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid);
    static std::vector<odtc::Obstacle<2>> GetObstaclesFromROSMsg(const image_pipeline_msgs::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid, std::map<odtc::TrackId, odtc::RegistrationData>& trackId2WorldFRegData_);
    static std::vector<odtc::Obstacle<2>> GetTracksFromROSMsg(const image_pipeline_msgs::msg::ObstacleArray &msg);

    static bool ReadROSObstacleArray(const message_filters::Cache<image_pipeline_msgs::msg::Obstacles> &cache, const rclcpp::Time t,
      image_pipeline_msgs::msg::Obstacles::ConstPtr &obstacles, const double maxTimeLag_s);
    static image_pipeline_msgs::msg::Obstacles FillObstaclesMsg(rclcpp::Time t, const std::vector<Buoy> &b,
                                                                  const std::vector<Marker> &m, const std::vector<Number> &n,
                                                                  const std::vector<Pipe> &p, const ctb::LatLong &centroid,
                                                                  const Eigen::TransformationMatrix &worldF_T_vehicleF);

  static geographic_msgs::msg::GeoPoseWithCovariance EigenToGeoPoseWithCovariance(const Eigen::TransformationMatrix& eigen_pose, const ctb::LatLong &centroid);
  static image_pipeline_msgs::msg::Buoy BuoyToBuoyMsg(const Buoy& buoy, const ctb::LatLong &centroid);
  static image_pipeline_msgs::msg::Number NumberToNumberMsg(const Number& number, const ctb::LatLong &centroid);
  static image_pipeline_msgs::msg::Marker MarkerToMarkerMsg(const Marker& marker, const ctb::LatLong &centroid);
  static image_pipeline_msgs::msg::Pipe PipeToPipeMsg(const Pipe& pipe, const ctb::LatLong &centroid);

  static ObstaclesData ObstaclesMsgToObstacles(const image_pipeline_msgs::msg::Obstacles &msg);
  static Eigen::TransformationMatrix GeoPoseWithCovarianceToEigen(const geographic_msgs::msg::GeoPoseWithCovariance &geo_pose_msg, const ctb::LatLong &centroid);

  static Buoy BuoyMsgToBuoy(const image_pipeline_msgs::msg::Buoy &msg, const ctb::LatLong &centroid);
  static Marker MarkerMsgToMarker(const image_pipeline_msgs::msg::Marker &msg, const ctb::LatLong &centroid);
  static Number NumberMsgToNumber(const image_pipeline_msgs::msg::Number &msg, const ctb::LatLong &centroid);
  static Pipe PipeMsgToPipe(const image_pipeline_msgs::msg::Pipe &msg, const ctb::LatLong &centroid);

  static std::vector<odtc::Obstacle<2>> ObstacleDataToObstacleVector(const ObstaclesData &obstacleData);

  static bool ReadBoxArray2DFromCache(const message_filters::Cache<image_pipeline_msgs::msg::BoundingBox2DArray> &cache,
    const rclcpp::Time stamp, image_pipeline_msgs::msg::BoundingBox2DArray::ConstPtr &msgPtr, const double maxTimeLag_s);
  
  static bool ReadPipeDirectionFromCache(const message_filters::Cache<image_pipeline_msgs::msg::PipeDirection> &cache,
    const rclcpp::Time stamp, image_pipeline_msgs::msg::PipeDirection::ConstPtr &msgPtr, const double maxTimeLag_s);
};

#endif
