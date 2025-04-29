#include "marine_detector_ros2.h"

#define PCD_BANK_PATH "/home/graal/data/clusters/"

using std::placeholders::_1;

MarineDetectorROS2::MarineDetectorROS2 (
    const std::string& bagPath, 
    const bool isSim
) : Node("marine_detector"), MarineDetector(bagPath, isSim) {
    if (enableBasicPrints_) {
        RCLCPP_INFO(this->get_logger(), "[MarineDetectorROS2::Init] Starting...");
    }

    Init(bagPath, isSim);

    InitSubscribers();
    InitPublishers();

    RCLCPP_INFO(this->get_logger(), "Settings message initialized successfully!");
    
    std::cerr << "[MarineDetectorROS2] mode is " << mode_ << std::endl;

    // Create synchronizer using message_filters
    if (mode_.find("RAMI") != std::string::npos) {
        std::cerr << "[MarineDetectorROS2] setting RAMI callbacks..." << std::endl;

        // Create the lidar subscription
        cameraObstaclesSub_ = this->create_subscription<obstacle_tracking_msg::msg::BoundingBox2DArray>(
            "/dtc/annotations/sf/AUV/rgb_camera",
            rclcpp::SensorDataQoS(),  // You can customize QoS here
            std::bind(&MarineDetectorROS2::ObstacleDetectionCallbackRAMI, this, std::placeholders::_1)
        );

        // Create the pose subscription
        geoPoseStampedSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            dsc_.topicImu,  // Replace with actual pose topic
            rclcpp::SensorDataQoS(),  // You can customize QoS here
            std::bind(&MarineDetectorROS2::SetWorldF_VehiclePose_GeoPoseStamped, this, std::placeholders::_1)
        );

        std::cerr << "[MarineDetectorROS2] setting callbacks END" << std::endl;
    
    }
    else {
	    throw std::runtime_error(std::string("[MarineDetectorROS2] " + mode_ + ": invalid mode").c_str());
    }

    fgRegionSub_= std::make_shared<message_filters::Subscriber<obstacle_tracking_msg::msg::BoundingBox2DArray>>(this, "/trk/collision_regions");
    cacheFgRegion_.setCacheSize(50);
    cacheFgRegion_.connectInput(*fgRegionSub_);

    // Create a timer with ROS2-style callback binding
    runTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / spinRate_),
        std::bind(&MarineDetectorROS2::Run, this)
    );
}

void MarineDetectorROS2::Init(const std::string& bagPath, bool isSim) {
    RCLCPP_INFO(this->get_logger(), "[MarineDetectorROS2::Init] Initializing with bagPath: %s, isSim: %s", 
                bagPath.c_str(), isSim ? "true" : "false");
    // Additional initialization logic here.
}

void MarineDetectorROS2::InitSubscribers() {
    std::cerr << std::endl << tc::bluL << "[InitSubscribers] Starting..." << tc::none << std::endl;
    dsc_.Print();

    // Initializing Camera Subscribers
    for (const auto &p : dsc_.cams) {
        imgSub_[p.first] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, dsc_.topicsImg[p.first]);
        std::cerr << "[MarineDetector] Detection topic for camera " << p.first << " is " << dsc_.topicsDetection[p.first] << std::endl;
        imgAnnSub_[p.first] = std::make_shared<message_filters::Subscriber<obstacle_tracking_msg::msg::BoundingBox2DArray>>(this, dsc_.topicsDetection[p.first]);
        cacheImg_[p.first] = std::make_shared<message_filters::Cache<sensor_msgs::msg::Image>>(25);
        cacheAnnotations_[p.first] = std::make_shared<message_filters::Cache<obstacle_tracking_msg::msg::BoundingBox2DArray>>(25);
        cacheAnnotations_[p.first]->connectInput(*imgAnnSub_[p.first]);
    }

    // Initializing Twist Subscriber
    twistSub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(this, dsc_.topicTwist);
    cacheVehicleTwist_.setCacheSize(20);
    cacheVehicleTwist_.connectInput(*twistSub_);

    // Initializing GNSS Subscriber
    gnssSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(this, dsc_.topicGNSS);
    cacheGNSS_.setCacheSize(20);
    cacheGNSS_.connectInput(*gnssSub_);

    // Initializing Tracks Subscriber
    tracksSub_ = std::make_shared<message_filters::Subscriber<obstacle_tracking_msg::msg::ObstacleArray>>(this, dsc_.topicTracks);
    cacheTracks_.setCacheSize(20);
    cacheTracks_.connectInput(*tracksSub_);

    std::cerr << tc::greenL << "[InitSubscribers] Finished!" << tc::none << std::endl;
};

void MarineDetectorROS2::InitPublishers() {
    RCLCPP_INFO(this->get_logger(), "[MarineDetectorROS2::InitPublishers] Setting up publishers.");
    // Initialize publishers
    settingsPub_ = this->create_publisher<obstacle_tracking_msg::msg::ObstDetectionSettings>("/dtc/detection_settings", 10);
    imuDataPub_ = this->create_publisher<sensor_msgs::msg::Imu>("/dtc/imu_data", 10);
    worldF_vehiclePosePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dtc/worldF_vehiclePose", 10);
    noCamSliceAnglesPub_ = this->create_publisher<obstacle_tracking_msg::msg::SliceAngles>("/dtc/no_cam_slice_angles", 10);
    worldF_cumulCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dtc/worldF_cumulCloud", 10);
    worldF_cumulCloud2DPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dtc/worldF_cumulCloud2D", 10);
    worldF_predictedTracksPub_ = this->create_publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>("/dtc/predictedTracks", 10);

    // Initialize sensor topics map for LiDARs
    sensorTopics st;
    st.worldF_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dtc/worldF_noCamCloud", 10);
    st.worldF_cloud2DPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dtc/worldF_noCamCloud2D", 10);
    st.worldF_hullsPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dtc/nocam/worldF_hulls", 10);
    st.worldF_obstacleBoundingBoxesPub = this->create_publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>("/dtc/nocam/worldF_boxes", 10);
    sensorsPub_["lidars"] = st;

    // Initialize sensor topics map for Cameras
    sensorTopics camst;
    camst.worldF_hullsPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dtc/cams/worldF_hulls", 10);
    camst.worldF_obstacleBoundingBoxesPub = this->create_publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>("/dtc/cams/worldF_boxes", 10);
    sensorsPub_["cams"] = camst;

    // Other publishers
    pyramidsPub_ = this->create_publisher<obstacle_tracking_msg::msg::PyramidArray>("/dtc/pyramids", 10);
    unfilteredPyramidsPub_ = this->create_publisher<obstacle_tracking_msg::msg::PyramidArray>("/dtc/pyramids_unfilt", 10);
    obstaclesPub_ = this->create_publisher<obstacle_tracking_msg::msg::ObstacleArray>("/dtc/worldF_obstacles", 10);
    statsPub_ = this->create_publisher<obstacle_tracking_msg::msg::ObstDetectionStats>("/dtc/stats", 10);
    sensorFovPub_ = this->create_publisher<obstacle_tracking_msg::msg::SliceAngles>("/dtc/sensorFoV", 1);

    imgFiltersPub_ = this->create_publisher<obstacle_tracking_msg::msg::ObstacleArrayMap>("/dtc/img_tracks", 10);
}

bool MarineDetectorROS2::SetWorldF_VehicleTwist() {
    if (firstRun_) worldF_vehicleTwist_.setZero();
    auto twistMsgPtr = cacheVehicleTwist_.getElemBeforeTime(tsRos_);
    if ((twistMsgPtr != nullptr) && UtilitiesROS2::TimestampsAreClose(tsRos_, twistMsgPtr->header.stamp, detection_.DtcParams().MaxMsgLag())) {
        worldF_vehicleTwist_ = UtilitiesROS2::ROSTwistToTwist(twistMsgPtr);
        if (enableRosDebugPrints_) std::cerr << tc::magL << "New vehicle twist = " << worldF_vehicleTwist_.transpose() << tc::none << std::endl;
        return true;
    }
    std::cerr << tc::yellow << "No new vehicle twist!" << tc::none << std::endl;
    return !firstRun_;
}

bool MarineDetectorROS2::SetPrevTrackInfo() {
    prevWorldF_tracks_.clear();
    obstacle_tracking_msg::msg::ObstacleArray::ConstPtr tracksPtr;
    auto tracksOk = UtilitiesROS2::ReadROSObstacleArray(cacheTracks_, tsRos_, tracksPtr, detection_.DtcParams().MaxMsgLag());
    if (tracksOk) {
        if (enableRosDebugPrints_)
            std::cerr << "[GetTracks] prev track t = " << UtilitiesROS2::ROSTimeToTimestamp(tracksPtr->header.stamp) - t0_ << std::endl;
        if (abs(UtilitiesROS2::ROSTimeToTimestamp(tracksPtr->header.stamp) - latestPrevTrackTs_) < 1e-3) tracksOk = false;
    }
    if (tracksOk) {
        auto worldF_obstacles =  UtilitiesROS2::GetTracksFromROSMsg(*tracksPtr);
        for (auto i = 0; i < worldF_obstacles.size(); i++) {
            if (tracksPtr->obstacles[i].id >= 0) {
                if (enableRosDebugPrints_)
                    std::cerr << "[GetTracks] filter id = " << tracksPtr->obstacles[i].id << ", size x is " << tracksPtr->obstacles[i].x.size() << std::endl;
                auto x = tracksPtr->obstacles[i].x;
                if (x.size() > 0) {
                    Eigen::Vector2d vel(x[5], x[6]);
                    prevWorldF_tracks_.emplace_back(odtc::TrackInfo(*worldF_obstacles[i].Box(), vel, UtilitiesROS2::ROSTimeToTimestamp(tracksPtr->header.stamp)));
                }
            }
        }
         if (enableRosDebugPrints_) std::cerr << tc::none << "Read " << prevWorldF_tracks_.size() << " tracks, dt = "
            << UtilitiesROS2::ROSTimeToTimestamp(tsRos_) - UtilitiesROS2::ROSTimeToTimestamp(tracksPtr->header.stamp) << "s , diff prev sample is "
            << UtilitiesROS2::ROSTimeToTimestamp(tracksPtr->header.stamp) - latestPrevTrackTs_ << " s." << tc::none << std::endl;
        
        latestPrevTrackTs_ = UtilitiesROS2::ROSTimeToTimestamp(tracksPtr->header.stamp);
        return true;
    }
    else {
        if (enableRosDebugPrints_) std::cerr << tc::yellow << "No new tracks! " << tc::none << std::endl;
        return false;
    }
}

bool MarineDetectorROS2::SetWorldF_VehiclePosition() {
    auto fixPtr = cacheGNSS_.getElemBeforeTime(tsRos_);
    if ((fixPtr != nullptr) && UtilitiesROS2::TimestampsAreClose(tsRos_, fixPtr->header.stamp, detection_.DtcParams().MaxMsgLag())) {
        llh_vehiclePos_ = Eigen::Vector3d(fixPtr->latitude, fixPtr->longitude, fixPtr->altitude);
        if (!firstGNSSReceived_) {
            std::cerr << tc::greenL << "[SetWorldF_VehiclePosition] Lat Long 0 set! It's " << llh_vehiclePos_.transpose() << tc::none << std::endl;
            llh_vehiclePos_t0_ = llh_vehiclePos_;
        }
        Eigen::Vector3d worldF_O_vehicleF;
        ctb::LatLong2LocalUTM(ctb::LatLong(llh_vehiclePos_[0], llh_vehiclePos_[1]), llh_vehiclePos_[2],
                              ctb::LatLong(llh_vehiclePos_t0_[0], llh_vehiclePos_t0_[1]), worldF_O_vehicleF);
        //worldF_O_vehicleF[2] = 1.5;
        if (datasetName_.find("mooring_field") != std::string::npos && datasetName_.find("dusk_mooring") == std::string::npos) {
            worldF_O_vehicleF[1] = -worldF_O_vehicleF[1];
            worldF_T_vehicleF_.TranslationVector(worldF_O_vehicleF); // TODO something?
            std::cerr << "[SetWorldF_VehiclePosition] Rev" << std::endl;
        }
        else worldF_T_vehicleF_.TranslationVector(worldF_O_vehicleF);
        /*if (enableRosDebugPrints_) {
            std::cerr << tc::magL << "[SetWorldF_VehiclePosition] Diff latlong = "<< (llh_vehiclePos_ - llh_vehiclePos_t0_).transpose() << tc::none << std::endl;
        }*/
        std::cerr << tc::none << "[SetWorldF_VehiclePosition] New vehicle GNSS Fix = " << llh_vehiclePos_.transpose() << tc::none << std::endl;
        std::cerr << tc::none << "[SetWorldF_VehiclePosition] New vehicle ENU Pos = " << worldF_T_vehicleF_.TranslationVector().transpose() << tc::none << std::endl;
        return true;
    }
    std::cerr << tc::yellow << "[SetWorldF_VehiclePosition] No new vehicle GNSS Fix!" << tc::none << std::endl;
    return false;
}

bool MarineDetectorROS2::SetFGRegions() {
    fgRegions_.clear(); 
    auto regPtr = cacheFgRegion_.getElemBeforeTime(tsRos_);
    if ((regPtr != nullptr) && (UtilitiesROS2::TimestampsAreClose(tsRos_, regPtr->header.stamp, detection_.DtcParams().MaxMsgLag()))) {
        if (abs(UtilitiesROS2::ROSTimeToTimestamp(regPtr->header.stamp) - latestFGRegionTs_) > 1e-4) {
            for (const auto &r : regPtr->boxes) {
                fgRegions_.emplace_back(UtilitiesROS2::GetBox2DFromMsg(r));
            }
            std::cerr << tc::greenL << "[SetFGRegions] New fg reg list, size is " << fgRegions_.size() << tc::none << std::endl;
            latestFGRegionTs_ = UtilitiesROS2::ROSTimeToTimestamp(regPtr->header.stamp);
            return true;
        }
    }
    std::cerr << tc::redL << "[SetFGRegions] NO fg reg list" << tc::none << std::endl;
    return false;
}

bool MarineDetectorROS2::SetAnnotations(std::string camId) {
    auto annPtrAft = cacheAnnotations_[camId]->getElemAfterTime(tsRos_);
    auto annPtrBef = cacheAnnotations_[camId]->getElemBeforeTime(tsRos_);
    auto annPtr = annPtrBef;
    if ((annPtr != nullptr)) {// && UtilitiesROS2::TimestampsAreClose(tsRos_, annPtr->header.stamp, detection_.DtcParams().MaxMsgLag())) {
        if (abs(UtilitiesROS2::ROSTimeToTimestamp(annPtr->header.stamp) - cam2LatestAnnotationTs_[camId]) > 1e-4) {
            std::cerr << tc::yellow << "[SetAnnotations] Diff is " << abs(UtilitiesROS2::ROSTimeToTimestamp(annPtr->header.stamp) - cam2LatestAnnotationTs_[camId]) << tc::none << std::endl;
            imgAnnotations_[camId].clear();
            cam2LatestAnnotationTs_[camId] = UtilitiesROS2::ROSTimeToTimestamp(annPtr->header.stamp);
            for (const auto &boxMsg : annPtr->boxes) {
                auto box = UtilitiesROS2::GetBox2DFromMsg(boxMsg);
                imgAnnotations_[camId].push_back(box);
            }
            return true;
        }
    }
    return false;
}

bool MarineDetectorROS2::SetWorldF_VehicleOrientation() {
	throw std::runtime_error(std::string("[MrDetROS2::SetWorldF_VehicleOrientation] Not implemented!").c_str());
}

bool MarineDetectorROS2::SetWorldF_VehicleOrientationIMU(const sensor_msgs::msg::Imu::ConstSharedPtr& imuMsg) {
    worldF_T_vehicleF_.block(0,0,3,3).setIdentity();
    if ((imuMsg != nullptr) && UtilitiesROS2::TimestampsAreClose(tsRos_, imuMsg->header.stamp, detection_.DtcParams().MaxMsgLag())) {
        worldF_T_vehicleF_.RotationMatrix(UtilitiesROS2::ROSImuMsgToRotation(imuMsg));
        if (enableRosDebugPrints_) std::cerr << tc::magL << "New vehicle orientation (YPR) = " << worldF_T_vehicleF_.RotationMatrix().ToEulerRPY() << tc::none << std::endl;
        return true;
    }
    std::cerr << tc::yellow << "No new vehicle orientation!" << tc::none << std::endl;
    return false;
}
bool MarineDetectorROS2::SetWorldF_VehiclePose_GeoPoseStamped(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg) {
    if (odometry_msg != nullptr) {
        // Extract pose from Odometry message
        auto pose = odometry_msg->pose.pose;

        // Extract position and orientation
        auto position = pose.position;
        auto orientation = pose.orientation;

        // Convert position to Eigen::Vector3d
        llh_vehiclePos_ = Eigen::Vector3d(position.x, position.y, position.z);

        // Handle initial GNSS reception
        if (!firstGNSSReceived_) {
            std::cerr << tc::greenL << "[SetWorldF_VehiclePose_Odometry] Lat Long 0 set! It's " 
                      << llh_vehiclePos_.transpose() << tc::none << std::endl;
            llh_vehiclePos_t0_ = llh_vehiclePos_;
        }

        // Convert to local frame (NED or UTM)
        Eigen::Vector3d worldF_O_vehicleF;
        if (frameType_ == odtc::FrameType::NED) {
            ctb::LatLong2LocalNED(ctb::LatLong(llh_vehiclePos_[0], llh_vehiclePos_[1]), llh_vehiclePos_[2],
                                  ctb::LatLong(llh_vehiclePos_t0_[0], llh_vehiclePos_t0_[1]), worldF_O_vehicleF);
        } else {
            ctb::LatLong2LocalUTM(ctb::LatLong(llh_vehiclePos_[0], llh_vehiclePos_[1]), llh_vehiclePos_[2],
                                  ctb::LatLong(llh_vehiclePos_t0_[0], llh_vehiclePos_t0_[1]), worldF_O_vehicleF);
        }

        // Create transformation matrix
        Eigen::TransformationMatrix worldF_T_vehicleF;
        worldF_T_vehicleF.TranslationVector(worldF_O_vehicleF);

        // Convert orientation to rotation matrix
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        worldF_T_vehicleF.RotationMatrix(q.toRotationMatrix());

        // Store the pose in the queue
        worldF_poses_queue_.push(std::make_pair(odometry_msg->header.stamp, worldF_T_vehicleF));

        std::cerr << tc::greenL << "[SetWorldF_VehiclePose_Odometry] Vehicle pose received!" << tc::none << std::endl;
        return true;
    }

    // Log when no new message is received
    std::cerr << tc::yellow << "[SetWorldF_VehiclePose_Odometry] No new vehicle odometry data!" << tc::none << std::endl;
    return false;
}


void MarineDetectorROS2::ObstacleDetectionCallbackRAMI(const obstacle_tracking_msg::msg::BoundingBox2DArray::ConstSharedPtr& ann_msg) {
    tsRos_ = ann_msg->header.stamp;
    ts_ = UtilitiesROS2::ROSTimeToTimestamp(tsRos_);
    detection_.Ts(ts_);
    t0_ = detection_.T0();

    frameType_ = odtc::FrameType::NED;
    std::cerr << tc::greenL << "[ObstacleDetectionCallbackRAMI] Start, t = " << ts_ - t0_ << tc::none << std::endl;
    bool foundEgoPose = false;
    double dtLag_s;
    for (const auto& item : worldF_poses_queue_) {
        dtLag_s = abs(UtilitiesROS2::ROSTimeToTimestamp(ann_msg->header.stamp) - UtilitiesROS2::ROSTimeToTimestamp(item.first));
        //std::cerr << "dt_s = " << dtLag_s << std::endl;
        if (dtLag_s < 1e-1) {
            foundEgoPose = true;
            tsRos_ = item.first;
            worldF_T_vehicleF_ = item.second;
            break;
        }
    }
    if (foundEgoPose) {
        std::cerr << tc::none << "[ObstacleDetectionCallbackRAMI] New vehicle Geopose with fix = " << llh_vehiclePos_.transpose() << tc::none << std::endl;
        std::vector<odtc::BoundingBox<2>> imgBoxes_;
        for (const auto &bMsg : ann_msg->boxes) {
            auto b = UtilitiesROS2::GetBox2DFromMsg(bMsg);
            imgBoxes_.emplace_back(b);
            if (b.Description().find("bouy") != std::string::npos) {
                for (const auto &cam : dsc_.cams) {
                    odtc::Pyramid pyr(b, cam.second.ExtF_TP_imgPlaneF(), Eigen::Vector2d(0,0));
                    auto extF_sphereCenter = pyr.Get3DSphereCentroid(0.3, false);
                    std::cerr << tc::cyanL << "[ObstacleDetectionCallbackRAMI] Box label is " << b.Description() << " with confidence " << b.Confidence() << ", 3D pos is " <<
                        extF_sphereCenter.transpose() << std::endl;
                    break;
                }
            }
        }
    }
    else {
        std::cerr << tc::yellow << "[SetWorldF_VehiclePose_GeoPoseStamped] No pose found."<< tc::none <<std::endl;
        std::cerr << tc::greenL << "[ObstacleDetectionCallbackStonefish] Bad end!" << tc::none << std::endl;
        return;
    }

    firstGNSSReceived_ = true;

    std::cerr << tc::greenL << "[ObstacleDetectionCallbackStonefish] Finished!" << tc::none << std::endl;
    
}

void MarineDetectorROS2::FillSettingsMsg() {
    obstacle_tracking_msg::msg::ObstDetectionSettings settingsMsg;
    settingsMsg.spin_rate = spinRate_;
    settingsMsg.max_msg_lag = detection_.DtcParams().MaxMsgLag();
    settingsMsg.output_frame = odtc::FrameToString(detection_.DtcParams().OutputFrame());

    settingsMsg.img_detection_model = imgDetectionModel_;

    settingsMsg.track_dist_threshold = detection_.DtcParams().TrackingDistanceThreshold();
    //settingsMsg.trackingDt = detection_.DtcParams().TrackingDt();

    settingsMsg.num_threads_for_parallel_clustering = detection_.DtcParams().NThreads();

    for (const auto &c : dsc_.cams) {
        auto camName = c.first;
        auto cam = c.second;
        settingsMsg.cam_names.emplace_back(camName);

        std_msgs::msg::Float64MultiArray Pmsg;
        auto P = cam.CameraMatrix();
        //std::cerr << "P " << camName << " = " << std::endl << P << std::endl;
        Pmsg.data = std::vector<double>(P.data(), P.data() + P.size());
        Pmsg.layout.dim.resize(2);
        Pmsg.layout.dim[0].label = "rows";
        Pmsg.layout.dim[0].size = P.rows();
        Pmsg.layout.dim[1].label = "columns";
        Pmsg.layout.dim[1].size = P.cols();
        Pmsg.layout.data_offset = 0;
        settingsMsg.cam_p.emplace_back(Pmsg);

        std_msgs::msg::Float64MultiArray distortionMsg;
        distortionMsg.data = cam.Distortion();
        distortionMsg.layout.dim.resize(2);
        distortionMsg.layout.dim[0].label = "rows";
        distortionMsg.layout.dim[0].size = distortionMsg.data.size();
        distortionMsg.layout.dim[1].label = "columns";
        distortionMsg.layout.dim[1].size = 1;
        settingsMsg.cam_distortion.emplace_back(distortionMsg);

        std_msgs::msg::Float64MultiArray vehicleF_ORPY_camFMsg;
        auto vehicleF_O_camF = cam.ExtF_T_sensorF().TranslationVector();
        auto vehicleF_RPY_camF = cam.ExtF_T_sensorF().RotationMatrix().ToEulerRPY();
        vehicleF_ORPY_camFMsg.data = {
                                    vehicleF_O_camF[0], vehicleF_O_camF[1], vehicleF_O_camF[2], 
                                    vehicleF_RPY_camF.Roll(), vehicleF_RPY_camF.Pitch(), vehicleF_RPY_camF.Yaw() };
        vehicleF_ORPY_camFMsg.layout.dim.resize(2);
        vehicleF_ORPY_camFMsg.layout.dim[0].label = "rows";
        vehicleF_ORPY_camFMsg.layout.dim[0].size = vehicleF_ORPY_camFMsg.data.size();
        vehicleF_ORPY_camFMsg.layout.dim[1].label = "columns";
        vehicleF_ORPY_camFMsg.layout.dim[1].size = 1;
        settingsMsg.vehiclef_orpy_camf.emplace_back(vehicleF_ORPY_camFMsg);

        settingsMsg.cam_img_topics.emplace_back(dsc_.topicsImg[camName]);
        settingsMsg.cam_img_topics_compressed.emplace_back(dsc_.topicsImgCompressed[camName]);
        settingsMsg.cam_h.emplace_back(dsc_.cams[camName].ImageHeight());
        settingsMsg.cam_w.emplace_back(dsc_.cams[camName].ImageWidth());
    }
    
    settingsMsg_ = settingsMsg;
}

void MarineDetectorROS2::Run() {
    //RCLCPP_INFO(this->get_logger(), "[MarineDetectorROS2::Run] Running periodic logic.");
    // Add periodic processing logic here.
}
