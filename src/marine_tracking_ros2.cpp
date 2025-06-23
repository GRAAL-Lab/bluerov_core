
#include "marine_tracking_ros2.h"

MarineTrackingROS2::MarineTrackingROS2(const std::string& bagPath, const bool isSim) : Node("marine_detector") {
    filtersPub_ = this->create_publisher<auv_core_helper::msg::DtcList>("/detections", 10);
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
    tracker_.enableTrackPointChange = false;
    tracker_.enableFGR = false;
    tracker_.enablePointCoverage = false;
    tracker_.enableAdvTrackingPrint = false;
    runTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.1),
        std::bind(&MarineTrackingROS2::Run, this)
    );

    goodLabelMappings.insert( {objectNames::BUOY_NAME, objectNames::BUOY_NAME} );
    goodLabelMappings.insert( {objectNames::MAINPIPE_NAME, objectNames::MAINPIPE_NAME} );
    goodLabelMappings.insert( {objectNames::PIPESTRUCT_NAME, objectNames::PIPESTRUCT_NAME} );
    goodLabelMappings.insert( {objectNames::MARKER_NAME, objectNames::MARKER_NAME} );
    goodLabelMappings.insert( {objectNames::NUMBER_NAME, objectNames::NUMBER_NAME} );
    goodLabelMappings.insert( {objectNames::MANIPULATION_NAME, objectNames::MANIPULATION_NAME} );
    
    detectionsSub_ = std::make_shared<message_filters::Subscriber<image_pipeline_msgs::msg::Obstacles>>(this, "/dtc/obstacles");
    cacheDetections_.setCacheSize(100);
    cacheDetections_.connectInput(*detectionsSub_);

    std::cerr << tc::bluL << "[MarineTracking] Created" << tc::none << std::endl;
}

void MarineTrackingROS2::FiltersCallback(const image_pipeline_msgs::msg::Obstacles::ConstPtr& obstaclesMsg) {
    trackId2WorldFRegData_.clear();
    tracker_.enableDbgPrint = enableDbgPrint_;
    auto msgOk = SetTime(obstaclesMsg);
    if (enableDbgPrint_) std::cerr << std::endl << tc::bluL << "[FiltersCallback] Starting, t = " << ts_ - t0_ << tc::none << std::endl;

    auto t1_trk = std::chrono::steady_clock::now();
    auto t1_rcv = std::chrono::steady_clock::now();
    ObstaclesData obstacleData;
    if (msgOk) {
        obstacleData = UtilitiesROS2::ObstaclesMsgToObstacles(*obstaclesMsg);
    }
    auto t2_rcv = std::chrono::steady_clock::now();;
    auto dt_rcv_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_rcv-t1_rcv).count()/1000.0;
    if (enableDbgPrint_) std::cerr << "[FiltersCallback] Msg rcv dt = " << dt_rcv_ms << "ms @" << obstacleData.buoys.size() << " buoys detectied." << std::endl;

    auto obstacles = UtilitiesROS2::ObstacleDataToObstacleVector(obstacleData);

    auto t1_da = std::chrono::steady_clock::now();
    if (msgOk) tracker_.egoPose = UtilitiesROS2::ROSPoseToTransformMatrix(obstaclesMsg->worldf_pose_vehiclef);
    tracker_.t0_ = t0_;
    tracker_.t_ = ts_;
    tracker_.UpdateMeasurements(obstacles, goodLabelMappings);
    auto t2_da = std::chrono::steady_clock::now();
    auto dt_da_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_da-t1_da).count()/1000.0;
    if (enableDbgPrint_) std::cerr << "[FiltersCallback] Data assoc dt = " << dt_da_ms << "ms @" << tracker_.ObstacleMeasurements().size() << " detections, " << tracker_.Filters().size() << " filters." << std::endl;

    auto t1_fu = std::chrono::steady_clock::now();
    tracker_.UpdateFilters(odtc::FilteringStrategy::EKF, trackingDt_);
    auto t2_fu = std::chrono::steady_clock::now();;
    auto dt_fu_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_fu-t1_fu).count()/1000.0;
    if (enableDbgPrint_) std::cerr << "[FiltersCallback] Filter update dt = " << dt_fu_ms << "ms @" << tracker_.Filters().size() << " filters." << std::endl;

    auto t2_trk = std::chrono::steady_clock::now();
    auto dt_trk_ms = std::chrono::duration_cast<std::chrono::microseconds>(t2_trk-t1_trk).count()/1000.0;
    if (enableDbgPrint_) std::cerr << "[MarineTrackingROS2::Callback] Callback dt = " << dt_trk_ms << "ms" << std::endl;
    if (enableDbgPrint_) std::cerr << tc::bluL << "[MarineTrackingROS2::Callback] Finished!" << tc::none << std::endl;

    auto tracksMsg = UtilitiesROS2::FillObstacleArrayMsg(tsROS_, tracker_, TrackType::ENU, llh_vehiclePos_t0_, enableDbgPrint_);
    filtersPub_->publish(tracksMsg);
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
    image_pipeline_msgs::msg::Obstacles::ConstPtr obstaclesMsg;
    double dtLagMax = trackingDt_;
    if (isFirstMsg_) dtLagMax = std::numeric_limits<double>::max();
    auto oldTracksOk = UtilitiesROS2::ReadROSObstacleArray(cacheDetections_, cacheDetections_.getLatestTime() + rclcpp::Duration::from_seconds(2), obstaclesMsg, dtLagMax);
    FiltersCallback(obstaclesMsg);
    if (enableDbgPrint_) std::cerr << tc::bluL << "[MarineTrackingROS1::Run] Finished!" << tc::none << std::endl << std::endl;
    return;
}

bool MarineTrackingROS2::SetTime(const image_pipeline_msgs::msg::Obstacles::ConstPtr& obstacles) {
    bool messageIsValid;

    if (obstacles != nullptr) {
        auto currentTimestamp = UtilitiesROS2::ROSTimeToTimestamp(obstacles->header.stamp);
        messageIsValid = isFirstMsg_ || ((currentTimestamp > ts_) && (std::abs(currentTimestamp - ts_) > 1e-4));// && (std::abs(currentTimestamp - ts_) < trackingDt_*1.6));

        if (!(currentTimestamp > ts_)) {
           // std::cerr << tc::none << "[MarineTrackingROS1::SetTime] currentTimestamp <= ts_" << tc::none << std::endl;
        }
        if (!(std::abs(currentTimestamp - ts_) > 1e-4)) {
          //  std::cerr << tc::none << "[MarineTrackingROS1::SetTime] Difference between currentTimestamp and ts_ is not greater than 1e-4" << tc::none << std::endl;
        }
        if (!(std::abs(currentTimestamp - ts_) < trackingDt_*1.6)) {
            //std::cerr << tc::none << "[MarineTrackingROS1::SetTime] Difference between currentTimestamp and ts_ is not less than dt*1.6, it is " << std::abs(currentTimestamp - ts_) << tc::none << std::endl;
        }

        //messageIsValid = isFirstMsg_ || ((std::abs(currentTimestamp - ts_) > 1e-4) && (std::abs(currentTimestamp - ts_) < 0.2));
        if (messageIsValid) {
            //if (enableDbgPrint_)std::cerr << tc::bluL << "[MarineTrackingROS1::SetTime] Obstacle msg ok!" << tc::none << std::endl;
            ts_ = currentTimestamp;
            if (isFirstMsg_) t0_ = ts_;
            tsROS_ = UtilitiesROS2::TimestampToROSTime(ts_);
            isFirstMsg_ = false;
        }
    }
    else {
        //if (enableDbgPrint_)std::cerr << tc::none << "[MarineTrackingROS1::SetTime] Obstacles is null!" << tc::none << std::endl;
    }
    if (!isFirstMsg_) {
        enableDbgPrint_ = abs(tLastDbgPrint_ - ts_) > 1;
        if (enableDbgPrint_) tLastDbgPrint_ = ts_;
    }
    if (messageIsValid) return true;

    if (!firstRun_ && !isFirstMsg_) tsROS_ = tsROS_ + rclcpp::Duration::from_seconds(trackingDt_);
    else tsROS_ = UtilitiesROS2::TimestampToROSTime(t0_);
    ts_ = UtilitiesROS2::ROSTimeToTimestamp(tsROS_);
    if (enableDbgPrint_) std::cerr << tc::yellow << "[MarineTrackingROS1::SetTime] No obstacles msg received!" << tc::none << std::endl;
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
