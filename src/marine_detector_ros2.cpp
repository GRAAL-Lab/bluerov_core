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

        // Create the pose subscription
        geoPoseStampedSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            dsc_.topicImu,  // Replace with actual pose topic
            rclcpp::SensorDataQoS(),  // You can customize QoS here
            std::bind(&MarineDetectorROS2::PerceptionCallback, this, std::placeholders::_1)
        );

        std::cerr << "[MarineDetectorROS2] setting callbacks END" << std::endl;
    
    }
    else {
	    throw std::runtime_error(std::string("[MarineDetectorROS2] " + mode_ + ": invalid mode").c_str());
    }

    // Create a timer with ROS2-style callback binding
    runTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / spinRate_),
        std::bind(&MarineDetectorROS2::Run, this)
    );
}


void MarineDetectorROS2::GetPipe(const image_pipeline_msgs::msg::PipeDirection::SharedPtr msg) {
    // Process the PipeDirection message
    RCLCPP_INFO(this->get_logger(), "Received pipe direction!");
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
        std::cerr << "DTC TOPIC is " << dsc_.topicsDetection[p.first] << std::endl;
        imgAnnSub_ = std::make_shared<message_filters::Subscriber<image_pipeline_msgs::msg::BoundingBox2DArray>>(this, dsc_.topicsDetection[p.first]);
        imgAnnCache_.setCacheSize(20);
        imgAnnCache_.connectInput(*imgAnnSub_);
        break;
    }

    // Initializing GNSS Subscriber
    pipeSub_ = std::make_shared<message_filters::Subscriber<image_pipeline_msgs::msg::PipeDirection>>(this, "pipe_direction");
    pipeCache_.setCacheSize(20);
    pipeCache_.connectInput(*pipeSub_);

    // Initializing GNSS Subscriber
    gnssSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(this, dsc_.topicGNSS);
    cacheGNSS_.setCacheSize(20);
    cacheGNSS_.connectInput(*gnssSub_);

    std::cerr << tc::greenL << "[InitSubscribers] Finished!" << tc::none << std::endl;
};

void MarineDetectorROS2::InitPublishers() {
    RCLCPP_INFO(this->get_logger(), "[MarineDetectorROS2::InitPublishers] Setting up publishers.");
    // Initialize publishers
    settingsPub_ = this->create_publisher<image_pipeline_msgs::msg::ObstDetectionSettings>("/dtc/detection_settings", 10);
    imuDataPub_ = this->create_publisher<sensor_msgs::msg::Imu>("/dtc/imu_data", 10);
    obstaclesPub_ = this->create_publisher<image_pipeline_msgs::msg::Obstacles>("/dtc/obstacles", 10);
    statsPub_ = this->create_publisher<image_pipeline_msgs::msg::ObstDetectionStats>("/dtc/stats", 10);
}

bool MarineDetectorROS2::PerceptionCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg) {
    std::cerr << tc::greenL << "[Prcp] Start!" << tc::none << std::endl;
    if (odometry_msg != nullptr) {
        tsRos_ = odometry_msg->header.stamp;
        ts_ = UtilitiesROS2::ROSTimeToTimestamp(tsRos_);
        detection_.Ts(ts_);
        t0_ = detection_.T0();

        std::cerr << tc::cyanL << "[Prcp] odometry_msg not nullptr" << tc::none << std::endl;
        image_pipeline_msgs::msg::BoundingBox2DArray::ConstPtr ann_msg;
        auto yoloDetectionsReceived = UtilitiesROS2::ReadBoxArray2DFromCache(imgAnnCache_, odometry_msg->header.stamp, ann_msg, 0.1); // TODO parameterize
        std::cerr << tc::cyanL << "[ObstacleDetectionCallbackRAMI] yoloDetectionsReceived = " << yoloDetectionsReceived << tc::none << std::endl;

        // and other topics
        if (!yoloDetectionsReceived) return false;

        std::vector<Buoy> buoys;
        std::vector<Marker> markers;
        std::vector<Number> numbers;
        std::vector<Pipe> pipes;
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

        if (yoloDetectionsReceived) {
            std::cerr << tc::none << "[ObstacleDetectionCallbackRAMI] New vehicle Geopose with fix = " << llh_vehiclePos_.transpose() << tc::none << std::endl;
            std::vector<odtc::BoundingBox<2>> imgBoxes_;
            for (const auto &bMsg : ann_msg->boxes) {
                auto b = UtilitiesROS2::GetBox2DFromMsg(bMsg);
                imgBoxes_.emplace_back(b);
                if (b.Description().find("buoy") != std::string::npos) {
                    for (const auto &cam : dsc_.cams) {
                        double buoyRadius = 0.3; // TODO will depend on color
                        odtc::Pyramid pyr(b, cam.second.ExtF_TP_imgPlaneF(), Eigen::Vector2d(0,0));
                        auto extF_sphereCenter = pyr.Get3DSphereCentroid(0.3, false);
                        std::cerr << tc::cyanL << "[ObstacleDetectionCallbackRAMI] Box label is " << b.Description() << " with confidence " << b.Confidence() << ", 3D pos is " <<
                            extF_sphereCenter.transpose() << std::endl;
                        Eigen::TransformationMatrix wF_buoyPose;
                        wF_buoyPose.TranslationVector(extF_sphereCenter);
                        Buoy b;
                        b.color = "red"; // TODO fill correctly
                        b.id = 0; // TODO boh
                        b.radius = buoyRadius;
                        b.wF_pose = wF_buoyPose;
                        b.notes = "";
                        buoys.emplace_back(b);
                        break;
                    }
                }
            }
        }

        firstGNSSReceived_ = true;
        auto obstaclesMsg = UtilitiesROS2::FillObstaclesMsg(tsRos_, buoys, markers, numbers, pipes, ctb::LatLong(llh_vehiclePos_t0_[0], llh_vehiclePos_t0_[1]));
        obstaclesPub_->publish(obstaclesMsg);
    
        std::cerr << tc::greenL << "[ObstacleDetectionCallbackStonefish] Finished!" << tc::none << std::endl;

        std::cerr << tc::greenL << "[SetWorldF_VehiclePose_Odometry] Vehicle pose received!" << tc::none << std::endl;
        return true;
    }

    // Log when no new message is received
    std::cerr << tc::yellow << "[SetWorldF_VehiclePose_Odometry] No new vehicle odometry data!" << tc::none << std::endl;
    return false;
}

void MarineDetectorROS2::FillSettingsMsg() {
    image_pipeline_msgs::msg::ObstDetectionSettings settingsMsg;
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
