
#include "marine_tracking_ros2.h"

MarineTrackingROS2::MarineTrackingROS2(const std::string& bagPath, const bool isSim) : Node("marine_detector") {
    filtersPub_ = this->create_publisher<obstacle_tracking_msg::msg::ObstacleArray>("/trk/tracks", 10);
    likelyCollisionRegionsPub_ = this->create_publisher<obstacle_tracking_msg::msg::BoundingBox2DArray>("/trk/collision_regions", 10);
    std::map<std::string, odtc::IDAssocParams> assocParams;
    odtc::TrackingParams trackingParams;

    libconfig::Config confObj;
    ReadConfigFile(bagPath.c_str(), confObj);
    libconfig::Setting &root = confObj.getRoot();
    std::string detectionParamsPath;
    root.lookup("params").lookupValue("detectionParams", detectionParamsPath);
    root.lookup("sim").lookupValue("t0", t0_);

    std::cerr << "detectionParamsPath = " << detectionParamsPath << std::endl;
    libconfig::Config confObjParams;
    ReadConfigFile(detectionParamsPath.c_str(), confObjParams);
    libconfig::Setting &rootParams = confObjParams.getRoot();

    ReadTrackingParams(trackingParams, assocParams, rootParams);
    
    root.lookup("dataset").lookupValue("name", datasetName_);
	std::cout << tc::magL << "[MarineTracking::Init] Dataset name: " << datasetName_ << tc::none << std::endl;

    tracker_ = odtc::Tracking(trackingParams,assocParams);
    tracker_.Meas2Track().Print();
    tracker_.enableTrackPointChange = true;
    tracker_.enableFGR = true;
    runTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(trackingDt_),
        std::bind(&MarineTrackingROS2::Run, this)
    );
    
    detectionsSub_ = std::make_shared<message_filters::Subscriber<obstacle_tracking_msg::msg::ObstacleArray>>(this, "/dtc/worldF_obstacles");
    cacheDetections_.setCacheSize(100);
    cacheDetections_.connectInput(*detectionsSub_);

    std::cerr << tc::bluL << "[MarineTracking] Created" << tc::none << std::endl;
}

void MarineTrackingROS2::FiltersCallback(const obstacle_tracking_msg::msg::ObstacleArray::ConstPtr& obstaclesMsg) {
    trackId2WorldFRegData_.clear();
    auto msgOk = SetTime(obstaclesMsg);
    std::cerr << tc::bluL << "[FiltersCallback] Starting, t = " << ts_ - t0_ << tc::none << std::endl;

    auto t1_trk = std::chrono::steady_clock::now();
    auto t1_rcv = std::chrono::steady_clock::now();
    std::vector<odtc::Obstacle<2>> obstacles;
    if (msgOk) {
        obstacles = UtilitiesROS2::GetObstaclesFromROSMsg(*obstaclesMsg, llh_vehiclePos_t0_, trackId2WorldFRegData_);
    }
    if (trackId2WorldFRegData_.size() > 0) {
        for (const auto &r : trackId2WorldFRegData_) {
            double dHeading = r.second.T.RotationMatrix().ToEulerRPY().Yaw();
            std::cerr << tc::yellow << "[FiltersCallback] trackId2WorldFRegData_ --> ID " << r.first << ", dt is " << r.second.dt << ", angle change is " << dHeading << tc::none << std::endl; // DEBUG, TODO REMOVE
        }
        tracker_.id2regData_ = trackId2WorldFRegData_;
    } // TODO reorganize
    std::cerr << std::endl;
    auto t2_rcv = std::chrono::steady_clock::now();;
    auto dt_rcv_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_rcv-t1_rcv).count()/1000.0;
    std::cerr << "[FiltersCallback] Msg rcv dt = " << dt_rcv_ms << "ms @" << obstacles.size() << " detections." << std::endl;

    auto t1_da = std::chrono::steady_clock::now();
    if (msgOk) tracker_.egoPose = UtilitiesROS2::ROSPoseToTransformMatrix(obstaclesMsg->worldf_pose_vehiclef);
    auto trackerCopy = tracker_;
    trackerCopy.UpdateMeasurements(obstacles);
    auto t2_da = std::chrono::steady_clock::now();
    auto dt_da_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_da-t1_da).count()/1000.0;
    std::cerr << "[FiltersCallback] Data assoc dt = " << dt_da_ms << "ms @" << trackerCopy.ObstacleMeasurements().size() << " detections, " << trackerCopy.Filters().size() << " filters." << std::endl;

    auto t1_fu = std::chrono::steady_clock::now();
    trackerCopy.UpdateFilters(odtc::FilteringStrategy::EKF, trackingDt_);
    auto t2_fu = std::chrono::steady_clock::now();;
    auto dt_fu_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_fu-t1_fu).count()/1000.0;
    std::cerr << "[FiltersCallback] Filter update dt = " << dt_fu_ms << "ms @" << trackerCopy.Filters().size() << " filters." << std::endl;

    auto obstaclesRevised = trackerCopy.ReviseMeasurements(obstacles); // TODO revise
    std::cerr << "enableDetectionRevision_ = "<< enableDetectionRevision_ << std::endl;
    if (!enableDetectionRevision_) obstaclesRevised = obstacles;

    auto t1_da2 = std::chrono::steady_clock::now();
    tracker_.UpdateMeasurements(obstaclesRevised);
    auto t2_da2 = std::chrono::steady_clock::now();
    auto dt_da2_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_da2-t1_da2).count()/1000.0;
    std::cerr << "[FiltersCallback] Data assoc dt = " << dt_da2_ms << "ms @" << tracker_.ObstacleMeasurements().size() << " detections 2, " << tracker_.Filters().size() << " filters." << std::endl;

    auto t1_fu2 = std::chrono::steady_clock::now();
    tracker_.UpdateFilters(odtc::FilteringStrategy::EKF, trackingDt_);
    auto t2_fu2= std::chrono::steady_clock::now();;
    auto dt_fu2_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_fu2-t1_fu2).count()/1000.0;
    std::cerr << "[FiltersCallback] Filter update dt = " << dt_fu2_ms << "ms @" << tracker_.Filters().size() << " filters." << std::endl;

    for (const auto &o : obstaclesRevised) {
        //std::cerr << "[TRKK] Obstacle revised: id is " << o.Box()->Id() << ", label is " << o.Box()->Description() << std::endl;
    }
    if (obstaclesRevised.size() != obstacles.size()) {
        std::cerr << tc::bluL << "[FiltersCallback] !!!! Obstacle size: " <<  obstacles.size() << " --> " << obstaclesRevised.size()  << tc::none << std::endl;
    }
    std::vector<odtc::BoundingBox<2>> boxesRevised;
    for (auto i = 0; i  < obstaclesRevised.size(); i++) {
        auto o = obstaclesRevised[i];
        auto bx = *o.Box();
        bx.Id(i);
        boxesRevised.emplace_back(bx);
    }
    auto tracksMsg = UtilitiesROS2::FillObstacleArrayMsg(tsROS_, tracker_, TrackType::ENU, llh_vehiclePos_t0_, boxesRevised);
    filtersPub_->publish(tracksMsg);

    std::vector<odtc::BoundingBox<2>> fgrBoxes;
    for (const auto &r : tracker_.likelyCollisionRegions_) fgrBoxes.emplace_back(r.c);
    UtilitiesROS2::PublishBoundingBoxes2D(likelyCollisionRegionsPub_, tsROS_, fgrBoxes);

    firstRun_ = false;
    auto t2_trk = std::chrono::steady_clock::now();
    auto dt_trk_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_trk-t1_trk).count()/1000.0;
    std::cerr << "[MarineTrackingROS2::Callback] Callback dt = " << dt_trk_ms << "ms" << std::endl;
    std::cerr << tc::bluL << "[MarineTrackingROS2::Callback] Finished!" << tc::none << std::endl;

}
void MarineTrackingROS2::ReadTrackingParams(odtc::TrackingParams &trackingParams, std::map<std::string, odtc::IDAssocParams> &assocParams,
    libconfig::Setting &rootParams) {

    double tempDouble;
    int tempInt;

    trackingParams.M = 3;
    trackingParams.N = 5;
    trackingParams.distNoisy = 90.0;
    trackingParams.lenThrNoisy = 0.1;
    trackingParams.widthThrNoisy = 0.1;

    // Reading Qdiag from libconfig
    std::vector<double> Qdiag;
    auto &Qdiag_node = rootParams.lookup("Qdiag");
    for (auto &q : Qdiag_node) Qdiag.emplace_back(q);
    trackingParams.Q.resize(Qdiag.size(), Qdiag.size());
    trackingParams.Q.setZero();
    for (auto idx = 0; idx < Qdiag.size(); idx++) { trackingParams.Q(idx, idx) = Qdiag[idx]; }
    std::cerr << "[ReadTrackingParams] Qdiag size = " << Qdiag.size() << std::endl << std::endl;
    std::cerr << "[ReadTrackingParams] Q = "<< std::endl << trackingParams.Q << std::endl;

    trackingParams.Q_noisy.resize(Qdiag.size(), Qdiag.size());
    trackingParams.Q_noisy.setZero();
    trackingParams.Q_noisy.diagonal().setConstant(1000.0);

    // Reading Rdiag from libconfig
    std::vector<double> Rdiag;
    auto &Rdiag_node = rootParams.lookup("Rdiag");
    for (auto &r : Rdiag_node) Rdiag.emplace_back(r);
    trackingParams.R0.resize(Rdiag.size(), Rdiag.size());
    trackingParams.R0.setZero();
    for (auto idx = 0; idx < Rdiag.size(); idx++) { trackingParams.R0(idx, idx) = Rdiag[idx]; }
    std::cerr << "[ReadTrackingParams] Rdiag size = " << Rdiag.size() << std::endl;
    std::cerr << "[ReadTrackingParams] R0 = "<< std::endl << trackingParams.R0 << std::endl;

    std::vector<double> R0_distDiag;
    auto &R0_distDiag_node = rootParams.lookup("R0Diag_dist");
    for (auto &r : R0_distDiag_node) R0_distDiag.emplace_back(r);
    std::cerr << "[ReadTrackingParams] R0Diag_dist size = " << R0_distDiag.size() << std::endl;
    trackingParams.R0_dist.resize(odtc::ekfMeasDim,odtc::ekfMeasDim);
    trackingParams.R0_dist = Eigen::MatrixXd::Zero(R0_distDiag.size(), R0_distDiag.size());
    for (auto idx = 0; idx < R0_distDiag.size(); idx++) { trackingParams.R0_dist(idx, idx) = R0_distDiag[idx]; }
    std::cerr << "[ReadTrackingParams] R0Diag_dist = "<< std::endl << trackingParams.R0_dist << std::endl;

    std::vector<double> R0_fragDiag;
    auto &R0_fragDiag_node = rootParams.lookup("R0Diag_frag");
    for (auto &r : R0_fragDiag_node) R0_fragDiag.emplace_back(r);
    std::cerr << "[ReadTrackingParams] R0Diag_frag size = " << R0_fragDiag.size() << std::endl;
    trackingParams.R0_dist.resize(odtc::ekfMeasDim,odtc::ekfMeasDim);
    trackingParams.R0_frag = Eigen::MatrixXd::Zero(R0_fragDiag.size(), R0_fragDiag.size());
    for (auto idx = 0; idx < R0_fragDiag.size(); idx++) { trackingParams.R0_frag(idx, idx) = R0_fragDiag[idx]; }
    std::cerr << "[ReadTrackingParams] R0Diag_frag = "<< std::endl << trackingParams.R0_frag << std::endl;

    int intTemp;
    intTemp = rootParams.lookup("enableRAEKF");
    
    intTemp = rootParams.lookup("enableAdaptiveR");
    trackingParams.enableAdaptiveR = intTemp;
    intTemp = rootParams.lookup("enableYFrag");
    trackingParams.enableY_frag = intTemp;
    intTemp = rootParams.lookup("enableYDist");
    trackingParams.enableY_dist = intTemp;
    intTemp = rootParams.lookup("enableYRot");
    trackingParams.enableY_rot = intTemp;
    std::cerr << "[ReadTrackingParams] enableAdaptiveR = "<< std::endl << trackingParams.enableAdaptiveR << std::endl;
    std::cerr << "[ReadTrackingParams] enableY_dist = "<< std::endl << trackingParams.enableY_dist << std::endl;

    intTemp = rootParams.lookup("enableDetectionRevision");
    enableDetectionRevision_ = intTemp;

    // Reading weightIoU
    tempDouble = rootParams.lookup("weightIoU");
    assocParams[odtc::Str(odtc::Metric::IOU)].weight = tempDouble;
    assocParams[odtc::Str(odtc::Metric::IOU)].maxAbs = 0.95;
    assocParams[odtc::Str(odtc::Metric::IOU)].sat = 100;

    // Reading weightCentroidDist
    tempDouble = rootParams.lookup("weightCentroidDist");
    assocParams[odtc::Str(odtc::Metric::DIST)].weight = tempDouble;
    assocParams[odtc::Str(odtc::Metric::DIST)].maxAbs = 10;
    assocParams[odtc::Str(odtc::Metric::DIST)].sat = 30;

    // Reading weightSimilarity
    tempDouble = rootParams.lookup("weightSimilarity");
    assocParams[odtc::Str(odtc::Metric::SIMILARITY)].weight = tempDouble;
    assocParams[odtc::Str(odtc::Metric::SIMILARITY)].maxAbs = 10;
    assocParams[odtc::Str(odtc::Metric::SIMILARITY)].sat = 30;

    // Reading weightSize
    tempDouble = rootParams.lookup("weightSize");
    assocParams[odtc::Str(odtc::Metric::SIZE)].weight = tempDouble;
    assocParams[odtc::Str(odtc::Metric::SIZE)].maxAbs = 1000;
    assocParams[odtc::Str(odtc::Metric::SIZE)].sat = 30;

    // Reading maxNumOcclusions
    tempInt = rootParams.lookup("maxNumOcclusions");
    trackingParams.maxOccl = tempInt;
    std::cerr << "[ReadTrackingParams] Max n occl = " << trackingParams.maxOccl << std::endl;

    trackingParams.maxOcclNoisy = 3;
    trackingParams.maxOcclTentative= 3;
    trackingParams.maxOcclZombie = 3;

    // Reading trackingDt
    trackingDt_ = rootParams.lookup("trackingDt");
    std::cerr << "[ReadTrackingParams] trackingDt = " << trackingDt_ << std::endl;
}

void MarineTrackingROS2::Run() {
    std::cerr << tc::bluL << "[MarineTrackingROS1::Run] Start..." << tc::none << std::endl;
    obstacle_tracking_msg::msg::ObstacleArray::ConstPtr obstaclesMsg;
    double dtLagMax = trackingDt_;
    if (isFirstMsg_) dtLagMax = std::numeric_limits<double>::max();
    std::cerr << "[Run] now = " << UtilitiesROS2::ROSTimeToTimestamp(this->get_clock()->now()) << std::endl;
    std::cerr << "[Run] cache detection latest = " << UtilitiesROS2::ROSTimeToTimestamp(cacheDetections_.getLatestTime()) << std::endl;
    auto oldTracksOk = UtilitiesROS2::ReadROSObstacleArray(cacheDetections_, cacheDetections_.getLatestTime() + rclcpp::Duration::from_seconds(2), obstaclesMsg, dtLagMax);
    FiltersCallback(obstaclesMsg);
    std::cerr << tc::bluL << "[MarineTrackingROS1::Run] Finished!" << tc::none << std::endl << std::endl;
    return;
}

bool MarineTrackingROS2::SetTime(const obstacle_tracking_msg::msg::ObstacleArray::ConstPtr& obstacles) {
    bool messageIsValid;

    if (obstacles != nullptr) {
        auto currentTimestamp = UtilitiesROS2::ROSTimeToTimestamp(obstacles->header.stamp);
        messageIsValid = isFirstMsg_ || ((currentTimestamp > ts_) && (std::abs(currentTimestamp - ts_) > 1e-4));// && (std::abs(currentTimestamp - ts_) < trackingDt_*1.6));

        if (!(currentTimestamp > ts_)) {
            std::cerr << tc::none << "[MarineTrackingROS1::SetTime] currentTimestamp <= ts_" << tc::none << std::endl;
        }
        if (!(std::abs(currentTimestamp - ts_) > 1e-4)) {
            std::cerr << tc::none << "[MarineTrackingROS1::SetTime] Difference between currentTimestamp and ts_ is not greater than 1e-4" << tc::none << std::endl;
        }
        if (!(std::abs(currentTimestamp - ts_) < trackingDt_*1.6)) {
            //std::cerr << tc::none << "[MarineTrackingROS1::SetTime] Difference between currentTimestamp and ts_ is not less than dt*1.6, it is " << std::abs(currentTimestamp - ts_) << tc::none << std::endl;
        }

        //messageIsValid = isFirstMsg_ || ((std::abs(currentTimestamp - ts_) > 1e-4) && (std::abs(currentTimestamp - ts_) < 0.2));
        if (messageIsValid) {
            std::cerr << tc::bluL << "[MarineTrackingROS1::SetTime] Obstacle msg ok!" << tc::none << std::endl;
            ts_ = currentTimestamp;
            tsROS_ = UtilitiesROS2::TimestampToROSTime(ts_);
            isFirstMsg_ = false;
            return true;
        }
    }
    else {
        std::cerr << tc::none << "[MarineTrackingROS1::SetTime] Obstacles is null!" << tc::none << std::endl;
    }

    if (!firstRun_ && !isFirstMsg_) tsROS_ = tsROS_ + rclcpp::Duration::from_seconds(trackingDt_);
    else tsROS_ = UtilitiesROS2::TimestampToROSTime(t0_);
    ts_ = UtilitiesROS2::ROSTimeToTimestamp(tsROS_);
    std::cerr << tc::yellow << "[MarineTrackingROS1::SetTime] No obstacles msg received!" << tc::none << std::endl;
    return false;
}


void MarineTrackingROS2::ReadConfigFile(const std::string& dataPath, libconfig::Config& confObj) {
    try {
        confObj.readFile(dataPath.c_str());
        std::cout << "Config file read successfully." << std::endl;
    }
    catch (const libconfig::FileIOException &fioex) {
        std::cerr << "I/O error while reading file: " << dataPath << std::endl;
    }
    catch (const libconfig::ParseException &pex) {
        std::cerr << "Parse error at " << pex.getFile() << ": " << pex.getLine()
                  << " - " << pex.getError() << std::endl;
    }
    catch (const std::exception &ex) {
        std::cerr << "An unexpected error occurred: " << ex.what() << std::endl;
    }
}
