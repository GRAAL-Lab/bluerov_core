#include <utilities_ros2.h>

double UtilitiesROS2::ROSTimeToTimestamp(const rclcpp::Time stamp) {
    return stamp.nanoseconds() * pow(10,-9);
}

rclcpp::Time UtilitiesROS2::TimestampToROSTime(const double timestamp) {
    double seconds_part;
    double fractional_part = std::modf(timestamp, &seconds_part);
    int64_t seconds = static_cast<int64_t>(seconds_part);
    uint32_t nanoseconds = static_cast<uint32_t>(fractional_part * 1e9);
    return rclcpp::Time(seconds, nanoseconds);
}

bool UtilitiesROS2::TimestampsAreClose(const rclcpp::Time stamp1, const rclcpp::Time stamp2, const double tol) {
    return abs(ROSTimeToTimestamp(stamp1) - ROSTimeToTimestamp(stamp2) <= tol);
}

Eigen::TransformationMatrix ROSPoseToTFMatrix(const geometry_msgs::msg::PoseStamped::ConstPtr& imuMsg) {
    auto pos = imuMsg->pose.position;
    auto rot = imuMsg->pose.orientation;
    
    Eigen::TransformationMatrix result;
    result.TranslationVector(Eigen::Vector3d(pos.x,pos.y,pos.z));
    result.RotationMatrix(Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z).toRotationMatrix());
    return result;
}

Eigen::RotationMatrix UtilitiesROS2::ROSImuMsgToRotation(const sensor_msgs::msg::Imu::ConstPtr& imuMsg) {
    return Eigen::Quaterniond(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z).toRotationMatrix();
}

Eigen::Vector6d UtilitiesROS2::ROSTwistToTwist(const geometry_msgs::msg::TwistStamped::ConstPtr twistMsg) {
    Eigen::Vector6d v;
    v << twistMsg->twist.linear.x, twistMsg->twist.linear.y, twistMsg->twist.linear.z,
        twistMsg->twist.angular.x, twistMsg->twist.angular.y, twistMsg->twist.angular.z;
    return v;
}

odtc::BoundingBox<2> UtilitiesROS2::GetBox2DFromMsg(const image_pipeline_msgs::msg::BoundingBox2D &boxMsg) {
    // Extract information from the BoundingBox2D message
    Eigen::Vector2d center(boxMsg.center_x, boxMsg.center_y);
    Eigen::Vector2d size(boxMsg.size_x, boxMsg.size_y);
    
    // Assuming the Yaw and ID are directly usable
    double yaw = boxMsg.yaw;
    rml::EulerRPY RPYbox(0,0,yaw);
    // Reconstruct the BoundingBox using the data from the BoundingBox2D message
    odtc::BoundingBox<2> box(center, RPYbox.ToRotationMatrix().block(0,0,2,2), size);
    
    // Set additional properties from the message
    box.Id(boxMsg.id);
    box.Description(boxMsg.desc);
    box.Confidence(boxMsg.conf);

    return box;
}

image_pipeline_msgs::msg::BoundingBox2D UtilitiesROS2::GetBox2DMsg(const rclcpp::Time t, const odtc::BoundingBox<2> box) {
    
    image_pipeline_msgs::msg::BoundingBox2D boxMsg;
    boxMsg.header.stamp = t;

    Eigen::Vector2d center = box.ExtF_Center();
    boxMsg.center_x = center[0];
    boxMsg.center_y = center[1];

    Eigen::Vector2d size = box.Sizes();
    boxMsg.size_x = size[0];
    boxMsg.size_y = size[1];

    std::cerr << "[GetBox2DMsg] id = " << box.Id() << ", size = " << boxMsg.size_x << ", " << boxMsg.size_y << " @" << center.transpose() << std::endl;

    boxMsg.yaw = box.Yaw();
    boxMsg.id = box.Id();

    boxMsg.desc = box.Description();
    boxMsg.conf = box.Confidence();

    return boxMsg;
}

void UtilitiesROS2::PublishBoundingBoxes2D(const rclcpp::Publisher<image_pipeline_msgs::msg::BoundingBox2DArray>::SharedPtr pub,
                                           const rclcpp::Time &t,
                                           const std::vector<odtc::BoundingBox<2>> &boxes) {
    // Define the message
    image_pipeline_msgs::msg::BoundingBox2DArray msg;

    // Set the header
    msg.header.stamp = t;
    msg.header.frame_id = "frame_id"; // Update frame_id as needed

    // Fill the bounding boxes
    for (const auto &box : boxes) {
        msg.boxes.emplace_back(GetBox2DMsg(t, box));
        //std::cerr << tc::bluL << "[PublishBoundingBoxes2D] len is " << msg.boxes[msg.boxes.size() - 1].size_x << tc::none << std::endl;
    }

    // Publish the message
    pub->publish(msg);
}

void UtilitiesROS2::PublishStats(const rclcpp::Publisher<image_pipeline_msgs::msg::ObstDetectionStats>::SharedPtr pub, const rclcpp::Time t,
                                            std::map<std::string, odtc::ClusteringStats> &stats) {
    image_pipeline_msgs::msg::ObstDetectionStats msg;
    msg.header.stamp = t;

    for (const auto &p : stats) {
        auto s = p.second;

        msg.sensor_name.emplace_back(p.first);

        msg.n_boxes.emplace_back(s.nBoxes);
        msg.n_points.emplace_back(s.nPoints);
        msg.n_dilation_points.emplace_back(s.nDilationPoints);
        msg.n_clusters.emplace_back(s.nClusters);

        msg.clustering_runtime_ms.emplace_back(s.clusteringRuntime_ms);
        msg.bb_gen_runtime_ms.emplace_back(s.bbGenRuntime_ms);
        msg.box_dilation_runtime_ms.emplace_back(s.boxDilationRuntime_ms);
        msg.preprocessing_runtime_ms.emplace_back(s.preprocessingRuntime_ms);
    }

    pub->publish(msg);
}


void UtilitiesROS2::PublishPose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub, const rclcpp::Time t,
                                            const Eigen::TransformationMatrix T, std::string frames) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frames;

    auto position = T.TranslationVector();
    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];

    auto quaternion = T.RotationMatrix().ToQuaternion();
    msg.pose.orientation.w = quaternion.w();
    msg.pose.orientation.x = quaternion.x();
    msg.pose.orientation.y = quaternion.y();
    msg.pose.orientation.z = quaternion.z();

    pub->publish(msg);
}

bool UtilitiesROS2::ReadBoxArray2DFromCache(const message_filters::Cache<image_pipeline_msgs::msg::BoundingBox2DArray> &cache,
    const rclcpp::Time stamp, image_pipeline_msgs::msg::BoundingBox2DArray::ConstPtr &msgPtr, const double maxTimeLag_s) {
    
    msgPtr = cache.getElemBeforeTime(stamp);
    return (msgPtr != nullptr) && (TimestampsAreClose(stamp, msgPtr->header.stamp, maxTimeLag_s));
}

bool UtilitiesROS2::ReadImageFromCache(const message_filters::Cache<sensor_msgs::msg::Image> &imgCache, const rclcpp::Time stamp, sensor_msgs::msg::Image::ConstPtr &imageMsgPtr, const double maxTimeLag_s) {
    imageMsgPtr = imgCache.getElemBeforeTime(stamp);
    std::cerr << "imageMsgPtr is null?" << (imageMsgPtr == nullptr) << std::endl;
    return (imageMsgPtr != nullptr) && (TimestampsAreClose(stamp, imageMsgPtr->header.stamp, maxTimeLag_s));
}

image_pipeline_msgs::msg::Obstacle UtilitiesROS2::FillObstacleMsg(rclcpp::Time t, std::shared_ptr<odtc::BoundingBox<2>> box,
              std::shared_ptr<odtc::TrackData> filterInfo, const std::map<std::string, odtc::IDAssocParams> &assocParams) {
    
    image_pipeline_msgs::msg::Obstacle msg;
    msg.header.stamp = t;

    if (filterInfo != nullptr) {
        auto f = filterInfo->f;
        auto id = filterInfo->id;
        auto numOcclusions = filterInfo->numOcclusions;
        msg.id = id;
        msg.n_occlusions = numOcclusions;
        msg.state = filterInfo->GetState();
        auto state = f.StateVector();
        auto P = f.PropagationError();
        msg.p = std::vector<double>(P.data(), P.data() + P.rows() * P.cols());;
        msg.x = std::vector<double>(state.data(), state.data() + state.rows() * state.cols());

        auto worldF_centroid = filterInfo->trackCentroid;
        msg.x[odtc::ekfStatePosxIdx] = worldF_centroid[0];
        msg.x[odtc::ekfStatePosyIdx] = worldF_centroid[1];
        if (filterInfo->A_dist.cols() > 0) {
            Eigen::VectorXd A_dist_diag = filterInfo->A_dist.diagonal();
            Eigen::VectorXd R0_dist_diag = filterInfo->R0_dist.diagonal();
            msg.r0_dist = std::vector<double>(R0_dist_diag.data(), R0_dist_diag.data() + R0_dist_diag.rows() * R0_dist_diag.cols());
            msg.a_dist = std::vector<double>(A_dist_diag.data(), A_dist_diag.data() + A_dist_diag.rows() * A_dist_diag.cols());
        }
        if (filterInfo->A_frag.cols() > 0) {
            Eigen::VectorXd A_frag_diag = filterInfo->A_frag.diagonal();
            Eigen::VectorXd R0_frag_diag = filterInfo->R0_frag.diagonal();
            msg.r0_frag = std::vector<double>(R0_frag_diag.data(), R0_frag_diag.data() + R0_frag_diag.rows() * R0_frag_diag.cols());
            msg.a_frag = std::vector<double>(A_frag_diag.data(), A_frag_diag.data() + A_frag_diag.rows() * A_frag_diag.cols());
        }
    }
    if (box != nullptr) {
        auto xRaw = odtc::TrackData::BoxToFilterState(*box);
      //  msg.x_raw = std::vector<double>(xRaw.data(), xRaw.data() + xRaw.rows() * xRaw.cols());
        msg.x_raw.resize(7);
        msg.x_raw[0] = xRaw[0];
        msg.x_raw[1] = xRaw[1];
        msg.x_raw[2] = xRaw[2];
        msg.x_raw[3] = xRaw[3];
        msg.x_raw[4] = xRaw[4];
        msg.x_raw[5] = xRaw[5];
        msg.x_raw[6] = xRaw[6];
        if (filterInfo != nullptr) { // velocities
            msg.x_raw[odtc::ekfStateVelxIdx] = filterInfo->f.StateVector()[odtc::ekfStatePosxIdx] - filterInfo->prevState[odtc::ekfStatePosxIdx];
            msg.x_raw[odtc::ekfStateVelyIdx] = filterInfo->f.StateVector()[odtc::ekfStatePosyIdx] - filterInfo->prevState[odtc::ekfStatePosyIdx];
        }
    }
    return msg;
}

image_pipeline_msgs::msg::ObstacleArray UtilitiesROS2::FillObstacleArrayMsg(rclcpp::Time t, std::vector<odtc::Obstacle<2>> &obstacles,
    std::vector<odtc::PolarRegion> &excludedRegions, const Eigen::Vector3d &llhCentroid, const Eigen::TransformationMatrix &worldF_T_vehicleF,
    std::map<size_t,std::vector<odtc::DetectionInfo>> di) {

 // std::cerr << "[FillObstacleArrayMsg] Start..." << std::endl;
    image_pipeline_msgs::msg::ObstacleArray msg;
    msg.header.stamp = t;
    for (int i = 0; i < obstacles.size(); i++) {
        //obstacles[i].SetBoundingBox(odtc::Cloud2BoxAlgorithm::HULL);
        auto boxPtr = obstacles[i].Box();
        auto center = boxPtr->ExtF_Center().head(2);

        auto detectionInfo = di[obstacles[i].Id()];
        double bestConfidence, bestArea;
        std::string bestLabel;
        bestConfidence = -1;
        for (auto diTmp : detectionInfo) {
           // std::cerr << diTmp.confidence <<" ";
            if (bestConfidence < diTmp.confidence) {
                bestConfidence = diTmp.confidence;
                bestLabel = diTmp.label;
                bestArea = diTmp.area;
            }
        }
        //std::cerr << "[FillObstacleArrayMsg] obstacle id = " << obstacles[i].Id() << " with label " << bestLabel << " and conf " << bestConfidence << std::endl;
        //std::cerr << std::endl;
        
        auto exclude = false;
        for (const auto &cloudRegion : excludedRegions) {
            auto dist = center.norm();
            auto inRange = (dist <= cloudRegion.rMax) && (dist >= cloudRegion.rMin);
            auto pointAngle = atan2(center[1], center[0]);
            auto inSlice = odtc::IsAngleIncluded(pointAngle, cloudRegion.thetaMin, cloudRegion.thetaMax);
            if (inSlice && inRange) {
            	exclude = true;
            	break;
            }
        }
        
        if (exclude) continue;
        
        auto obstacleMsg = FillObstacleMsg(t, boxPtr, nullptr, {});
        obstacleMsg.class_label = bestLabel;
        obstacleMsg.class_conf = bestConfidence;
        obstacleMsg.class_area = bestArea;
        obstacleMsg.meas_id = obstacles[i].Id();
        obstacles[i].SetHull();
        auto hull = *obstacles[i].PointsHull();
        for (int j = 0; j < hull.Size(); j++) {
            auto Q = hull.At(j);
            geometry_msgs::msg::Point P;
            P.x = Q[0];
            P.y = Q[1];
            if (Q.rows() == 3) P.z = Q[2];
            else P.z = 0.0;
            obstacleMsg.points.emplace_back(P);
        }
        msg.obstacles.emplace_back(obstacleMsg);
        msg.geo_centroid.position.latitude = llhCentroid[0];
        msg.geo_centroid.position.longitude = llhCentroid[1];
        msg.geo_centroid.position.altitude = llhCentroid[2];

        auto position = worldF_T_vehicleF.TranslationVector();
        msg.worldf_pose_vehiclef.position.x = position[0];
        msg.worldf_pose_vehiclef.position.y = position[1];
        msg.worldf_pose_vehiclef.position.z = position[2];

        auto quaternion = worldF_T_vehicleF.RotationMatrix().ToQuaternion();
        msg.worldf_pose_vehiclef.orientation.w = quaternion.w();
        msg.worldf_pose_vehiclef.orientation.x = quaternion.x();
        msg.worldf_pose_vehiclef.orientation.y = quaternion.y();
        msg.worldf_pose_vehiclef.orientation.z = quaternion.z();

    }
   // std::cerr << "[FillObstacleArrayMsg] End!" << std::endl;
    return msg;
}

Eigen::TransformationMatrix UtilitiesROS2::ROSPoseToTransformMatrix(const geometry_msgs::msg::Pose& msg) {
    // Extract position from the ROS message
    double x = msg.position.x;
    double y = msg.position.y;
    double z = msg.position.z;
    Eigen::Vector3d r(x, y, z);
    
    double qw = msg.orientation.w;
    double qx = msg.orientation.x;
    double qy = msg.orientation.y;
    double qz = msg.orientation.z;
    Eigen::Quaterniond quat(qw, qx, qy, qz);
    
    // Convert quaternion to rotation matrix
    Eigen::Matrix3d R = quat.toRotationMatrix();
    
    // Create the transformation matrix
    Eigen::TransformationMatrix T;
    T.TranslationVector(r);  // Set translation
    T.RotationMatrix(R);         // Set rotation
    
    return T;
}

Buoy UtilitiesROS2::BuoyMsgToBuoy(const image_pipeline_msgs::msg::Buoy &msg, const ctb::LatLong &centroid) {
    Buoy buoy;
    buoy.id = static_cast<uint64_t>(msg.id);
    buoy.wF_pose = GeoPoseWithCovarianceToEigen(msg.pose, centroid);
    buoy.color = msg.color;
    buoy.radius = msg.radius;
    buoy.notes = msg.notes;
    return buoy;
}

Marker UtilitiesROS2::MarkerMsgToMarker(const image_pipeline_msgs::msg::Marker &msg, const ctb::LatLong &centroid) {
    Marker marker;
    marker.id = static_cast<uint64_t>(msg.id);
    marker.wF_pose = GeoPoseWithCovarianceToEigen(msg.pose, centroid);
    marker.color = msg.color;
    marker.notes = msg.notes;
    return marker;
}

Number UtilitiesROS2::NumberMsgToNumber(const image_pipeline_msgs::msg::Number &msg, const ctb::LatLong &centroid) {
    Number number;
    number.id = static_cast<uint64_t>(msg.id);
    number.wF_pose = GeoPoseWithCovarianceToEigen(msg.pose, centroid);
    number.number = msg.number;
    number.bgColor = msg.bg_color;
    number.notes = msg.notes;
    return number;
}

Pipe UtilitiesROS2::PipeMsgToPipe(const image_pipeline_msgs::msg::Pipe &msg, const ctb::LatLong &centroid) {
    Pipe pipe;
    pipe.id = static_cast<uint64_t>(msg.id);
    pipe.wF_startPose = GeoPoseWithCovarianceToEigen(msg.start_pose, centroid);
    pipe.wF_endPose = GeoPoseWithCovarianceToEigen(msg.end_pose, centroid);
    pipe.wF_pose.TranslationVector(Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));

    // Convert markers
    for (const auto &marker_msg : msg.markers) {
        pipe.markers.emplace_back(MarkerMsgToMarker(marker_msg, centroid));
    }

    // Convert numbers
    for (const auto &number_msg : msg.numbers) {
        pipe.numbers.emplace_back(NumberMsgToNumber(number_msg, centroid));
    }

    pipe.notes = msg.notes;
    return pipe;
}

ObstaclesData UtilitiesROS2::ObstaclesMsgToObstacles(const image_pipeline_msgs::msg::Obstacles &msg) {
    ObstaclesData result;
    result.centroid.latitude = msg.centroid.position.latitude;
    result.centroid.longitude = msg.centroid.position.longitude;

    for (const auto &b_msg : msg.buoys) {
        result.buoys.emplace_back(BuoyMsgToBuoy(b_msg, result.centroid));
    }

    for (const auto &m_msg : msg.markers) {
        result.markers.emplace_back(MarkerMsgToMarker(m_msg, result.centroid));
    }

    for (const auto &p_msg : msg.pipes) {
        result.pipes.emplace_back(PipeMsgToPipe(p_msg, result.centroid));
    }

    for (const auto &n_msg : msg.numbers) {
        result.numbers.emplace_back(NumberMsgToNumber(n_msg, result.centroid));
    }

    return result;
}

std::vector<odtc::Obstacle<2>> UtilitiesROS2::ObstacleDataToObstacleVector(const ObstaclesData &obstacleData) {
    std::vector<odtc::Obstacle<2>> res;

    for (const auto &b : obstacleData.buoys) {
        odtc::BoundingBox<2> bx(b.wF_pose.TranslationVector().head(2), Eigen::Vector2d(b.radius * 2, b.radius * 2));
        bx.Id(b.id);
        res.emplace_back(bx);
    }

    return res;
}


image_pipeline_msgs::msg::Obstacles UtilitiesROS2::FillObstaclesMsg(rclcpp::Time t, const std::vector<Buoy> &buoys,
                                                                  const std::vector<Marker> &markers, const std::vector<Number> &numbers,
                                                                  const std::vector<Pipe> &pipes, const ctb::LatLong &centroid,
                                                                  const Eigen::TransformationMatrix &worldF_T_vehicleF) {
    image_pipeline_msgs::msg::Obstacles res;
    res.header.stamp = t;

    for (const auto &b : buoys) {
        res.buoys.emplace_back(BuoyToBuoyMsg(b, centroid));
    }

    for (const auto &m : markers) {
        res.markers.emplace_back(MarkerToMarkerMsg(m, centroid));
    }

    for (const auto &p : pipes) {
        res.pipes.emplace_back(PipeToPipeMsg(p, centroid));
    }

    for (const auto &n : numbers) {
        res.numbers.emplace_back(NumberToNumberMsg(n, centroid));
    }

    res.centroid.position.latitude = centroid.latitude;
    res.centroid.position.longitude = centroid.longitude;

    auto position = worldF_T_vehicleF.TranslationVector();
    res.worldf_pose_vehiclef.position.x = position[0];
    res.worldf_pose_vehiclef.position.y = position[1];
    res.worldf_pose_vehiclef.position.z = position[2];

    auto quaternion = worldF_T_vehicleF.RotationMatrix().ToQuaternion();
    res.worldf_pose_vehiclef.orientation.w = quaternion.w();
    res.worldf_pose_vehiclef.orientation.x = quaternion.x();
    res.worldf_pose_vehiclef.orientation.y = quaternion.y();
    res.worldf_pose_vehiclef.orientation.z = quaternion.z();

    return res;
}
#include <geographic_msgs/msg/geo_pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <Eigen/Dense>

// Assume Eigen::TransformationMatrix and Eigen::Vector3d are defined appropriately

Eigen::TransformationMatrix UtilitiesROS2::GeoPoseWithCovarianceToEigen(const geographic_msgs::msg::GeoPoseWithCovariance &geo_pose_msg, const ctb::LatLong &centroid) {
    Eigen::TransformationMatrix eigen_pose;

    // Convert position from GeoPose to Eigen format
    ctb::LatLong mapPoint;
    mapPoint.latitude = geo_pose_msg.pose.position.latitude;
    mapPoint.longitude = geo_pose_msg.pose.position.longitude;
    double altitude = geo_pose_msg.pose.position.altitude;

    Eigen::Vector3d position;
    ctb::LatLong2LocalNED(mapPoint, altitude, centroid, position);
    eigen_pose.TranslationVector(position);

    // Convert orientation from quaternion to Eigen rotation matrix
    Eigen::Quaterniond quaternion(
        geo_pose_msg.pose.orientation.w,
        geo_pose_msg.pose.orientation.x,
        geo_pose_msg.pose.orientation.y,
        geo_pose_msg.pose.orientation.z
    );
    eigen_pose.RotationMatrix() = quaternion.toRotationMatrix();

    return eigen_pose;
}

geographic_msgs::msg::GeoPoseWithCovariance UtilitiesROS2::EigenToGeoPoseWithCovariance(const Eigen::TransformationMatrix& eigen_pose, const ctb::LatLong &centroid) {
    geographic_msgs::msg::GeoPoseWithCovariance geo_pose_msg;

    // Extract position from eigen_pose and apply the NED centroid offset
    Eigen::Vector3d position = eigen_pose.TranslationVector();
    ctb::LatLong mapPoint;
    double altitude;
    ctb::LocalNED2LatLong(position, centroid, mapPoint, altitude);
    geo_pose_msg.pose.position.latitude = mapPoint.latitude; // Assuming x -> latitude (N)
    geo_pose_msg.pose.position.longitude = mapPoint.longitude; // Assuming y -> longitude (E)
    geo_pose_msg.pose.position.altitude = altitude; // Assuming z -> altitude (D)

    // Extract rotation from eigen_pose and convert to quaternion
    Eigen::Quaterniond quaternion(eigen_pose.RotationMatrix());
    geo_pose_msg.pose.orientation.x = quaternion.x();
    geo_pose_msg.pose.orientation.y = quaternion.y();
    geo_pose_msg.pose.orientation.z = quaternion.z();
    geo_pose_msg.pose.orientation.w = quaternion.w();

    // Covariance matrix setup
    geo_pose_msg.covariance.fill(0.0);
    for (int i = 0; i < 6; ++i) {
        geo_pose_msg.covariance[i * 6 + i] = 1.0;  // Identity covariance for simplicity
    }

    return geo_pose_msg;
}


image_pipeline_msgs::msg::Buoy UtilitiesROS2::BuoyToBuoyMsg(const Buoy& buoy, const ctb::LatLong &centroid) {
    image_pipeline_msgs::msg::Buoy msg;
    msg.id = static_cast<int64_t>(buoy.id);
    msg.pose = EigenToGeoPoseWithCovariance(buoy.wF_pose, centroid);
    msg.color = buoy.color;
    msg.radius = buoy.radius;
    msg.notes = buoy.notes;
    return msg;
}

// Number -> NumberMsg
image_pipeline_msgs::msg::Number UtilitiesROS2::NumberToNumberMsg(const Number& number, const ctb::LatLong &centroid) {
    image_pipeline_msgs::msg::Number msg;
    msg.id = static_cast<int64_t>(number.id);
    msg.pose = EigenToGeoPoseWithCovariance(number.wF_pose, centroid);
    msg.number = number.number;
    msg.bg_color = number.bgColor;
    msg.notes = number.notes;
    return msg;
}

// Marker -> MarkerMsg
image_pipeline_msgs::msg::Marker UtilitiesROS2::MarkerToMarkerMsg(const Marker& marker, const ctb::LatLong &centroid) {
    image_pipeline_msgs::msg::Marker msg;
    msg.id = static_cast<int64_t>(marker.id);
    msg.pose = EigenToGeoPoseWithCovariance(marker.wF_pose, centroid);
    msg.color = marker.color;
    msg.notes = marker.notes;
    return msg;
}

image_pipeline_msgs::msg::Pipe UtilitiesROS2::PipeToPipeMsg(const Pipe& pipe, const ctb::LatLong &centroid) {
    image_pipeline_msgs::msg::Pipe msg;
    msg.id = static_cast<int64_t>(pipe.id);
    msg.start_pose = EigenToGeoPoseWithCovariance(pipe.wF_startPose, centroid);
    msg.end_pose = EigenToGeoPoseWithCovariance(pipe.wF_endPose, centroid);
    msg.pose.position.x = pipe.wF_pose.TranslationVector()[0];
    msg.pose.position.y = pipe.wF_pose.TranslationVector()[1];
    msg.pose.position.z = pipe.wF_pose.TranslationVector()[2];
    //msg.pose.direction.x = ??
    //msg.pose.direction.y = ??
    //msg.pose.direction.z = ??

    // Convert markers
    for (const auto& marker : pipe.markers) {
        msg.markers.push_back(MarkerToMarkerMsg(marker, centroid));
    }

    // Convert numbers
    for (const auto& number : pipe.numbers) {
        msg.numbers.push_back(NumberToNumberMsg(number, centroid));
    }

    msg.notes = pipe.notes;
    return msg;
}

image_pipeline_msgs::msg::ObstacleArray UtilitiesROS2::FillObstacleArrayMsg(rclcpp::Time t, const odtc::Tracking &trck, const TrackType trackType, const Eigen::Vector3d &llhCentroid, std::vector<odtc::BoundingBox<2>> boxes) {

    image_pipeline_msgs::msg::ObstacleArray msg;
    msg.header.stamp = t;
    auto measObstacles = trck.ObstacleMeasurements();
    auto filters = trck.Filters();

    for (auto &f : filters) {
        size_t key;
        auto noOccl = futils::FindMapKeyByValue(trck.Meas2Track().A2B(), f.first, key);
        std::shared_ptr<odtc::BoundingBox<2U>> boxPtr = nullptr;
        if (noOccl) {
            if (measObstacles[key].BoxOk()) measObstacles[key].SetBoundingBox(odtc::Cloud2BoxAlgorithm::HULL);
            boxPtr = measObstacles[key].Box();
        }
        auto obstacleMsg = FillObstacleMsg(t, boxPtr, std::make_shared<odtc::TrackData>(f.second), trck.Meas2Track().Params());
        obstacleMsg.class_label = f.second.label;
        obstacleMsg.class_conf = f.second.labelConfidence;
        obstacleMsg.class_area = 0;// infoBox.Volume();

        if (noOccl) {
            auto cldPtr = measObstacles[key].Points();
            if (cldPtr != nullptr) {
                obstacleMsg.meas_id = key;
                for (int j = 0; j < cldPtr->Size(); j++) {
                    auto Q = cldPtr->At(j);
                    if (Q.size() >= 2) {
                        geometry_msgs::msg::Point P;
                        P.x = Q[0];
                        P.y = Q[1];
                        if (Q.rows() == 3) P.z = Q[2];
                        else P.z = 0.0;
                        obstacleMsg.points.emplace_back(P);
                        //std::cerr << "[FillObstacleArrayMsg] added point..." << std::endl;
                    }
                    else {
                        std::cerr << "[FillObstacleArrayMsg] null point..." << std::endl;
                    }
                }
            }
        }
        msg.obstacles.emplace_back(obstacleMsg);
    }
    auto idAssocParams = trck.Meas2Track().Params();
    auto trackingParams = trck.TrckParams();
    
    auto iouParams = idAssocParams[odtc::Str(odtc::Metric::IOU)];
    msg.metrics.emplace_back(odtc::Str(odtc::Metric::IOU));
    msg.metric_weight.emplace_back(iouParams.weight);
    msg.metric_max_abs.emplace_back(iouParams.maxAbs);
    msg.metric_sat.emplace_back(iouParams.maxAbs);

    auto distParams = idAssocParams[odtc::Str(odtc::Metric::DIST)];
    msg.metrics.emplace_back(odtc::Str(odtc::Metric::DIST));
    msg.metric_weight.emplace_back(distParams.weight);
    msg.metric_max_abs.emplace_back(distParams.maxAbs);
    msg.metric_sat.emplace_back(distParams.maxAbs);

    auto similParams = idAssocParams[odtc::Str(odtc::Metric::SIMILARITY)];
    msg.metrics.emplace_back(odtc::Str(odtc::Metric::SIMILARITY));
    msg.metric_weight.emplace_back(similParams.weight);
    msg.metric_max_abs.emplace_back(similParams.maxAbs);
    msg.metric_sat.emplace_back(similParams.maxAbs);

    msg.max_occl = trackingParams.maxOccl;

    std::vector<double> QVec(trackingParams.Q.data(), trackingParams.Q.data() + trackingParams.Q.rows() * trackingParams.Q.cols());
    msg.q = QVec;

    std::vector<double> RVec(trackingParams.R0.data(), trackingParams.R0.data() + trackingParams.R0.rows() * trackingParams.R0.cols());
    msg.r = RVec;

    return msg;
}

bool UtilitiesROS2::ReadROSObstacleArray(const message_filters::Cache<image_pipeline_msgs::msg::Obstacles> &cache, const rclcpp::Time t,
    image_pipeline_msgs::msg::Obstacles::ConstPtr &obstacles, const double maxTimeLag_s) {
        
    obstacles = cache.getElemBeforeTime(t);

    return (obstacles != nullptr) && (TimestampsAreClose(t, obstacles->header.stamp, maxTimeLag_s));
}

std::vector<odtc::Obstacle<2>> UtilitiesROS2::GetTracksFromROSMsg(const image_pipeline_msgs::msg::ObstacleArray &msg) {
    std::vector<odtc::Obstacle<2>> result;
    auto obstacles = msg.obstacles;
    obstacles.reserve(obstacles.size());
    Eigen::VectorXd state(5);
    for (auto i = 0; i < obstacles.size(); i++) {
        auto data = obstacles[i];
        if (data.x.size() >= 5) {
            state <<
                    data.x[odtc::ekfStatePosxIdx], data.x[odtc::ekfStatePosyIdx],
                    data.x[odtc::ekfStateLenIdx], data.x[odtc::ekfStateWidthIdx],
                    data.x[odtc::ekfStateYawIdx];
            auto box = odtc::TrackData::FilterStateToBoxStc(state);
            box.Id(data.id);
            std::vector<Eigen::Vector2d> points;
            for (const auto &P : data.points) {
                Eigen::Vector2d Peig(P.x, P.y);
                points.emplace_back(Peig);
            }
            odtc::Obstacle<2> obstacle(points);
            obstacle.Id(box.Id());
            obstacle.Box(box);
            obstacle.PointsHull(odtc::PointCloudHandler<2>(points));
            result.emplace_back(obstacle);
        }
    }
    //std::cerr << tc::greenL << "[UtilitiesROS2::TracksFromROSMsg] # track = " << result.size() << tc::none << std::endl;
    return result;
}

std::vector<odtc::Obstacle<2>> UtilitiesROS2::GetObstaclesFromROSMsg(const image_pipeline_msgs::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid) {
    std::map<odtc::TrackId, odtc::RegistrationData> unused;
    return GetObstaclesFromROSMsg(msg, geoCentroid, unused);
}

std::vector<odtc::Obstacle<2>> UtilitiesROS2::GetObstaclesFromROSMsg(const image_pipeline_msgs::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid, std::map<odtc::TrackId, odtc::RegistrationData>& trackId2WorldFRegData_) {
    std::vector<odtc::Obstacle<2>> result;
    auto obstacles = msg.obstacles;
    obstacles.reserve(obstacles.size());
    Eigen::VectorXd state(5);
    for (auto i = 0; i < obstacles.size(); i++) {
        auto data = obstacles[i];
        if (data.x_raw.size() >= 5) {
            state <<
                    data.x_raw[odtc::ekfStatePosxIdx], data.x_raw[odtc::ekfStatePosyIdx],
                    data.x_raw[odtc::ekfStateLenIdx], data.x_raw[odtc::ekfStateWidthIdx],
                    data.x_raw[odtc::ekfStateYawIdx];
            auto box = odtc::TrackData::FilterStateToBoxStc(state);
            box.Id(data.meas_id);
            box.Confidence(data.class_conf);
            box.Description(data.class_label);
            std::cerr << "[GetObstaclesFromROSMsg] obstacle id = " << box.Id() << " with label " << box.Description() << " and conf " << box.Confidence() << std::endl;
            std::vector<Eigen::Vector2d> points;
            for (const auto &P : data.points) {
                Eigen::Vector2d Peig(P.x, P.y);
                points.emplace_back(Peig);
                //std::cerr << "[GetObstaclesFromROSMsg] point added!" << std::endl;
            }
            odtc::PointCloudHandler<2> cld(points);
            odtc::Obstacle<2> obstacle(cld, box.Id(), "WorldF", box.Description(), box.Confidence());
            obstacle.Box(box);
            obstacle.PointsHull(odtc::PointCloudHandler<2>(points));
            auto bx = *obstacle.Box();
            //std::cerr << "[GetObstaclesFromROSMsg] Obst id = " << bx.Id() << " with label " << bx.Description() << " and conf " << bx.Confidence() << std::endl;
            result.emplace_back(obstacle);
        }
    }
    
    geoCentroid[0] = msg.geo_centroid.position.latitude;
    geoCentroid[1] = msg.geo_centroid.position.longitude;
    geoCentroid[2] = msg.geo_centroid.position.altitude;
    //std::cerr << tc::greenL << "[UtilitiesROS2::ObstaclesFromMsg] # obstacle = " << result.size() << tc::none << std::endl;
    return result;
}
