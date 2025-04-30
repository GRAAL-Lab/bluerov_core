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

odtc::BoundingBox<2> UtilitiesROS2::GetBox2DFromMsg(const obstacle_tracking_msg::msg::BoundingBox2D &boxMsg) {
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

obstacle_tracking_msg::msg::BoundingBox2D UtilitiesROS2::GetBox2DMsg(const rclcpp::Time t, const odtc::BoundingBox<2> box) {
    
    obstacle_tracking_msg::msg::BoundingBox2D boxMsg;
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

void UtilitiesROS2::PublishBoundingBoxes2D(const rclcpp::Publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>::SharedPtr pub,
                                           const rclcpp::Time &t,
                                           const std::vector<odtc::BoundingBox<2>> &boxes) {
    // Define the message
    obstacle_tracking_msg::msg::BoundingBox2DArray msg;

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

void UtilitiesROS2::PublishStats(const rclcpp::Publisher<obstacle_tracking_msg::msg::ObstDetectionStats>::SharedPtr pub, const rclcpp::Time t,
                                            std::map<std::string, odtc::ClusteringStats> &stats) {
    obstacle_tracking_msg::msg::ObstDetectionStats msg;
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

bool UtilitiesROS2::ReadImageFromCache(const message_filters::Cache<sensor_msgs::msg::Image> &imgCache, const rclcpp::Time stamp, sensor_msgs::msg::Image::ConstPtr &imageMsgPtr, const double maxTimeLag_s) {
    imageMsgPtr = imgCache.getElemBeforeTime(stamp);
    //std::cerr << "imageMsgPtr is null?" << (imageMsgPtr == nullptr) << std::endl;
    return (imageMsgPtr != nullptr && TimestampsAreClose(stamp, imageMsgPtr->header.stamp, maxTimeLag_s));
}

obstacle_tracking_msg::msg::Obstacle UtilitiesROS2::FillObstacleMsg(rclcpp::Time t, std::shared_ptr<odtc::BoundingBox<2>> box,
              std::shared_ptr<odtc::TrackData> filterInfo, const std::map<std::string, odtc::IDAssocParams> &assocParams) {
    
    obstacle_tracking_msg::msg::Obstacle msg;
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

obstacle_tracking_msg::msg::ObstacleArray UtilitiesROS2::FillObstacleArrayMsg(rclcpp::Time t, std::vector<odtc::Obstacle<2>> &obstacles,
    std::vector<odtc::PolarRegion> &excludedRegions, const Eigen::Vector3d &llhCentroid, const Eigen::TransformationMatrix &worldF_T_vehicleF,
    std::map<size_t,std::vector<odtc::DetectionInfo>> di) {

 // std::cerr << "[FillObstacleArrayMsg] Start..." << std::endl;
    obstacle_tracking_msg::msg::ObstacleArray msg;
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

detav_msgs::msg::ObstacleList UtilitiesROS2::FillDetavObstacleArrayMsg(rclcpp::Time t, const odtc::Tracking &trck, const Eigen::Vector3d &llhCentroid) {

    detav_msgs::msg::ObstacleList msg;
    msg.header.stamp = t;
    auto measObstacles = trck.ObstacleMeasurements();
    auto filters = trck.Filters();
    for (auto &f : filters) {
        size_t key;
        auto noOccl = futils::FindMapKeyByValue(trck.Meas2Track().A2B(), f.first, key);
        std::shared_ptr<odtc::BoundingBox<2U>> boxPtr = nullptr;
        if (noOccl) {
            //std::cerr << "[FillDetavObstacleArrayMsg] key is " << key << " and obstacle n is " << measObstacles.size() << std::endl;
            if (measObstacles[key].Box() == nullptr) measObstacles[key].SetBoundingBox(odtc::Cloud2BoxAlgorithm::HULL);
            boxPtr = measObstacles[key].Box();
        }
        auto obstacleMsg = FillDetavObstacleMsg(f.second, llhCentroid);
        msg.obstacles.emplace_back(obstacleMsg);
    }
    return msg;
}
detav_msgs::msg::Obstacle UtilitiesROS2::FillDetavObstacleMsg(const odtc::TrackData &tr, const Eigen::Vector3d &llhCentroid) {
    detav_msgs::msg::Obstacle res;
    res.id = std::to_string(tr.id);
    res.obs_class = tr.label;

    auto stateVec = tr.f.StateVector();
    auto stateCov = tr.f.PropagationError();

    // Calculate yawBest
    double vx = stateVec[odtc::ekfStateVelxIdx];
    double vy = stateVec[odtc::ekfStateVelyIdx];

    if (stateVec[odtc::ekfStateLenIdx] < stateVec[odtc::ekfStateWidthIdx]) {
        stateVec[odtc::ekfStateYawIdx] += M_PI_2;
        std::swap(stateVec[odtc::ekfStateLenIdx], stateVec[odtc::ekfStateWidthIdx]);
    }
    
    double yaw0 = stateVec[odtc::ekfStateYawIdx];
    double errBest = std::abs(std::atan2(vy, vx) - yaw0);
    double yawBest = yaw0;

    for (double dyaw : {M_PI}) {
        double yaw = yaw0 + dyaw;
        double errTmp = std::abs(std::atan2(vy, vx) - yaw);
        if (errTmp < errBest) {
            errBest = errTmp;
            yawBest = yaw;
        }
    }

    // Update yaw in stateVec
    stateVec[odtc::ekfStateYawIdx] = yawBest;

    Eigen::Vector3d obstaclePos(tr.trackCentroid[0], tr.trackCentroid[1], 0);

    ctb::LatLong obstaclePosLL;
    double obstacleAltitude;
    ctb::LatLong centroidLL(llhCentroid[0], llhCentroid[1]);
    ctb::LocalNED2LatLong(obstaclePos, centroidLL, obstaclePosLL, obstacleAltitude);
    res.pose.position.position.altitude = obstacleAltitude;
    res.pose.position.position.latitude = obstaclePosLL.latitude;
    res.pose.position.position.longitude = obstaclePosLL.longitude;

    res.pose.orientation.yaw = stateVec[odtc::ekfStateYawIdx];
    Eigen::Matrix6d poseCov = Eigen::Matrix6d::Zero();
    poseCov.block(0, 0, 2, 2) = stateCov.block(0, 0, 2, 2);
    poseCov.block(0, 5, 2, 1) = stateCov.block(0, 4, 2, 1);
    poseCov.block(5, 0, 1, 2) = stateCov.block(4, 0, 1, 2);
    poseCov(5, 5) = stateCov(4, 4);

    res.pose.position.north_cov = poseCov(0, 0);
    res.pose.position.east_cov = poseCov(1, 1);
    res.pose.orientation.covariance = poseCov(5, 5);

    res.size.size.length = stateVec[odtc::ekfStateLenIdx];
    res.size.size.width = stateVec[odtc::ekfStateWidthIdx];
    Eigen::Matrix2d sizeCov = Eigen::Matrix2d::Zero();
    sizeCov = stateCov.block(2, 2, 2, 2);
    res.size.covariance = {sizeCov(0, 0), sizeCov(1, 1)};

    res.twist.twist.linear.x = stateVec[odtc::ekfStateVelxIdx];
    res.twist.twist.linear.y = stateVec[odtc::ekfStateVelyIdx];
    res.twist.twist.linear.z = 0.0;
    Eigen::Matrix6d twistCov = Eigen::Matrix6d::Zero();
    twistCov.block(0, 0, 2, 2) = stateCov.block(5, 5, 2, 2);
    res.twist.covariance = {twistCov(0, 0), twistCov(1, 1), 0, 0, 0, 0};

    return res;
}

obstacle_tracking_msg::msg::ObstacleArray UtilitiesROS2::FillObstacleArrayMsg(rclcpp::Time t, const odtc::Tracking &trck, const TrackType trackType, const Eigen::Vector3d &llhCentroid, std::vector<odtc::BoundingBox<2>> boxes) {

    obstacle_tracking_msg::msg::ObstacleArray msg;
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

bool UtilitiesROS2::ReadROSObstacleArray(const message_filters::Cache<obstacle_tracking_msg::msg::ObstacleArray> &cache, const rclcpp::Time t,
    obstacle_tracking_msg::msg::ObstacleArray::ConstPtr &obstacles, const double maxTimeLag_s) {
        
    obstacles = cache.getElemBeforeTime(t);

    return (obstacles != nullptr) && (abs(UtilitiesROS2::ROSTimeToTimestamp(t) - UtilitiesROS2::ROSTimeToTimestamp(obstacles->header.stamp)) <= maxTimeLag_s);
}

std::vector<odtc::Obstacle<2>> UtilitiesROS2::GetTracksFromROSMsg(const obstacle_tracking_msg::msg::ObstacleArray &msg) {
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

std::vector<odtc::Obstacle<2>> UtilitiesROS2::GetObstaclesFromROSMsg(const obstacle_tracking_msg::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid) {
    std::map<odtc::TrackId, odtc::RegistrationData> unused;
    return GetObstaclesFromROSMsg(msg, geoCentroid, unused);
}

std::vector<odtc::Obstacle<2>> UtilitiesROS2::GetObstaclesFromROSMsg(const obstacle_tracking_msg::msg::ObstacleArray &msg, Eigen::Vector3d &geoCentroid, std::map<odtc::TrackId, odtc::RegistrationData>& trackId2WorldFRegData_) {
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
