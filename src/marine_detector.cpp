#include <marine_detector.h>

MarineDetector::MarineDetector(const std::string bagPath, const bool isSim) : isSim_(isSim) {
    Init(bagPath, isSim);
}

void MarineDetector::Init(const std::string dataPath, const bool isSim) {
    if (enableBasicPrints_) std::cerr << tc::bluL << "[MarineDetector::Init] Starting..." << tc::none << std::endl;

    libconfig::Config confObj;
    ReadConfigFile(dataPath.c_str(), confObj);
    libconfig::Setting &root = confObj.getRoot();
    std::string detectionParamsPath;
    root.lookup("params").lookupValue("detectionParams", detectionParamsPath);
    libconfig::Config confObjParams;
    ReadConfigFile(detectionParamsPath.c_str(), confObjParams);
    libconfig::Setting &rootParams = confObjParams.getRoot();
	std::cout << tc::magL << "[MarineDetector::Init] Next will read params..." << tc::none << std::endl;
    
    root.lookup("dataset").lookupValue("name", datasetName_);
	if (enableBasicPrints_) std::cout << tc::magL << "[MarineDetector::Init] Dataset name: " << datasetName_ << tc::none << std::endl;

    dsc_ = LoadTestConfig(root);
    dsc_.Print();

    odtc::ObstacleDetectionParameters dtcParams;
    odtc::Tracking imgTracker;
    ReadDetectionParams(rootParams, dtcParams, imgTracker);
    imgTracker.costTol = 300;
    
    detection_.Init(dtcParams, dsc_.cams, dsc_.lidars);
    
    for (auto const &c : dsc_.cams) {
        detection_.imgP_trackers_[c.first] = imgTracker;
    }

    double t0;
    root.lookup("sim").lookupValue("t0", t0);
    detection_.T0(t0);
    
    libconfig::Setting &rminRoot = root.lookup("sim.exclusionRegions_rMin");
    libconfig::Setting &rmaxRoot = root.lookup("sim.exclusionRegions_rMax");
    libconfig::Setting &thetaminRoot = root.lookup("sim.exclusionRegions_thetaMin");
    libconfig::Setting &thetamaxRoot = root.lookup("sim.exclusionRegions_thetaMax");
    int length = rminRoot.getLength();
    for (int i = 0; i < length; ++i) {
	double rmin = rminRoot[i];
	double rmax = rmaxRoot[i];
	double thetamin_deg = thetaminRoot[i];
	double thetamax_deg = thetamaxRoot[i];
    	odtc::PolarRegion cr(thetamin_deg * M_PI / 180.0, thetamax_deg * M_PI / 180.0, rmin, rmax);
    	detection_.AddExcludedRegion(cr);
   }

    int enableStandaloneLidar = 0;
    rootParams.lookupValue("enableStandaloneLidar", enableStandaloneLidar);
    enableStandaloneLidar_ = enableStandaloneLidar;
    int enableCamera = 0;
    rootParams.lookupValue("enableCamera", enableCamera);
    enableCamera_ = enableCamera;

    root.lookup("cam_detections").lookupValue("model", imgDetectionModel_);
    std::cerr << "[Detector] imgDetectionModel_ = " << imgDetectionModel_ << std::endl;
    root.lookup("cam_detections").lookupValue("model_ir", imgDetectionModel_ir_);
    std::cerr << "[Detector] imgDetectionModel_ir_ = " << imgDetectionModel_ir_ << std::endl;
    
    std::vector<std::pair<double,double>> vehicleF_camFoVs;
    double fov;
    for (const auto &p : dsc_.cams) {
        auto cam = p.second;
        auto vehicleF_fovBoundaries = cam.FoV(Eigen::Matrix4d::Identity(), fov);
        vehicleF_camFoVs.push_back(vehicleF_fovBoundaries);
        sensorName2FoV_[p.first] = vehicleF_fovBoundaries;
        std::cerr << tc::bluL << "[Detector] cam " << p.first << " -> vehF_FoV = " << vehicleF_fovBoundaries.first << "," << vehicleF_fovBoundaries.second << tc::none << std::endl;
    }
    sensorName2FoV_["lidars"] = std::make_pair(vehicleF_camFoVs[0].second, vehicleF_camFoVs[dsc_.cams.size() - 1].first);

    if ( enableStandaloneLidar && enableCamera) {
        detection_.AddVehicleFNoCamSlice(sensorName2FoV_["lidars"].first, sensorName2FoV_["lidars"].second);
    }
    else if (enableStandaloneLidar) {
        detection_.AddVehicleFNoCamSlice(0, 2 * M_PI);
    }

    if (enableRTAnnot_) {
    	
    }
    else {
    	for (const auto &c : dsc_.cams) {
	    std::cerr << tc::cyanL << "Cam --> " << c.first << tc::none << std::endl;
	    imgOfflineAnnotations_[c.first] = OfflineDetectionUtils::AnnotationFileToBoxVector(sensorName2AnnotationPath_[c.first]);
	}
    }
    
    firstRun_ = true;
    firstGNSSReceived_ = false;

    if (enableBasicPrints_) std::cerr << tc::greenL << "[MarineDetector::Init] Finished!" << tc::none << std::endl;
}


odtc::DetectionSensorConfiguration MarineDetector::LoadTestConfig(libconfig::Setting &testConfigSetting) {
    odtc::DetectionSensorConfiguration dsc;
    testConfigSetting.lookupValue("mode", mode_);

    // TOPICS
    std::vector<std::string> camTopics, camTopicsCompr, camDetectionTopics, lidarTopics;
    auto &topics = testConfigSetting.lookup("topics");
    dsc.topicImu = "/sf/bluerov/pose";
    dsc.topicTwist = "";
    dsc.topicGNSS = "";
    dsc.topicTracks = "trk/tracks";

    auto &sensors = testConfigSetting.lookup("params").lookup("sensors");
    int numSensors = sensors.getLength();
    std::cout << "[LoadTestConfig] Number of children in sensors: " << numSensors << std::endl;
    
    for (int i = 0; i < numSensors; ++i) {
        const libconfig::Setting& sensor = sensors[i];
        std::string sensorName = sensor.getName();
        auto cam = CreateCamera(sensor, 0, odtc::FrameType::NED);
        dsc.cams[sensorName] = cam;
        dsc.topicsImg[sensorName] = "/sf/AUV/rgb_camera";
        dsc.topicsDetection[sensorName] = "/dtc/annotations/sf/AUV/rgb_camera";
        std::cerr << "[LoadTestConfig] Sensor " + sensorName + " loaded!" << std::endl;
        //sensorName2AnnotationPath_[sensorName] = annotPath;
    }

    return dsc;
}

void MarineDetector::ReadConfigFile(const std::string& dataPath, libconfig::Config& confObj) {

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

odtc::Camera MarineDetector::CreateCamera(const libconfig::Setting& camSettings, const size_t camId, odtc::FrameType ft) {
    auto cameraName = std::string(camSettings.getName());
    std::vector<double> cameraProjectionMatrix, vehicleF_pose_cameraF, distortionCoeffs;
    int imageWidth, imageHeight;
    camSettings.lookupValue("image_width", imageWidth);
    camSettings.lookupValue("image_height", imageHeight);

    try {
    // Try to access the setting
    const libconfig::Setting& dc_node = camSettings.lookup("distortion_coefficients");
    
    // If found, process it
    for (auto &d : dc_node) {
        distortionCoeffs.emplace_back(d);
    }
    } catch (const libconfig::SettingNotFoundException &e) {
        distortionCoeffs = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
    double fx,fy,cx,cy;

    try {
        auto &P_node = camSettings.lookup("P");
        for (auto &P_el : P_node) cameraProjectionMatrix.emplace_back(P_el);
        fx = cameraProjectionMatrix.at(0);
        fy = cameraProjectionMatrix.at(5);
        cx = cameraProjectionMatrix.at(2);
        cy = cameraProjectionMatrix.at(6);
        std::cerr << tc::greenL << "P found! " << tc::none << std::endl;
    }
    catch (const libconfig::ConfigException& e) {
        double fov;
        camSettings.lookupValue("fov", fov);
        std::cerr << tc::yellow << "P not found, reading fov: " << fov << "deg." << tc::none << std::endl;
        auto P = odtc::ComputeProjectionMatrix(imageWidth, imageHeight, fov);
        fx = P(0,0);
        fy = P(1,1);
        cx = P(0,2);
        cy = P(1,2);
    }

    auto &vehicleF_pose_sensorF_node = camSettings.lookup("vehicleF_pose_sensorF");
    for (auto &x : vehicleF_pose_sensorF_node) vehicleF_pose_cameraF.emplace_back(x);
    Eigen::Vector3d vehicleF_cameraPosition = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vehicleF_pose_cameraF.data(), 3);
    rml::EulerRPY vehicleF_cameraOrientation(vehicleF_pose_cameraF[3], vehicleF_pose_cameraF[4], vehicleF_pose_cameraF[5]);
    
    odtc::Camera cam(camId, imageWidth, imageHeight, fx, fy, cx, cy, distortionCoeffs, vehicleF_cameraPosition, vehicleF_cameraOrientation, ft);
    return cam;
}

odtc::Lidar MarineDetector::CreateLidar(const libconfig::Setting& lidarSettings, const size_t lidarId, odtc::FrameType ft) {
    auto lidarName = std::string(lidarSettings.getName());
    std::vector<double> vehicleF_pose_lidarF;

    auto &vehicleF_pose_sensorF_node = lidarSettings.lookup("vehicleF_pose_sensorF");
    for (auto &x : vehicleF_pose_sensorF_node) vehicleF_pose_lidarF.emplace_back(x);
    Eigen::Vector3d vehicleF_lidarPosition = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vehicleF_pose_lidarF.data(), 3);
    rml::EulerRPY vehicleF_lidarOrientation(vehicleF_pose_lidarF[3], vehicleF_pose_lidarF[4], vehicleF_pose_lidarF[5]);

    odtc::Lidar lidar(lidarId, vehicleF_lidarPosition, vehicleF_lidarOrientation);
    return lidar;
}

void MarineDetector::InitOpenCVWindows(const unsigned int nCams) {
    for (auto i = 0; i < nCams; i++) {
        cv::namedWindow(opencvWindows_[i], 0);
        cv::resizeWindow(opencvWindows_[i],600,600);
    }
}

void MarineDetector::ReadDetectionParams(libconfig::Setting &root, odtc::ObstacleDetectionParameters &dtcParams, odtc::Tracking &imgTracker) {

    size_t tempSizeT; int tempInt; double tempDouble; std::string tempString;
    int minCluSize, maxCluSize, minPointsForCuda;
    int tempIntX, tempIntY;
    double cluTol, masatoQuality, dLength, dWidth;

    std::cerr << tc::bluL << "[ReadDetectionParams] Detection parameters configuration starting.." << tc::none << std::endl;

    //////////////// ROS NODE ///////////////
    root.lookupValue("spinRate", spinRate_);

    root.lookupValue("maxMsgLag", tempDouble);
    dtcParams.MaxMsgLag(tempDouble);

    //////////////// OUTPUT FRAME ////////////////
    std::string outputFrame;
    root.lookupValue("outputFrame", outputFrame);
    dtcParams.OutputFrame("World");

    //////////////// CLUSTERING PARAMETERS ////////////////
    root.lookupValue("cloudWindow", tempInt);
    std::cerr << "cloudWindow = " << tempInt << std::endl;
    dtcParams.HistoryLen(tempInt);

	root.lookupValue("clusterTolerance", cluTol);
	root.lookupValue("clusterMinSize", minCluSize);
	root.lookupValue("clusterMaxSize", maxCluSize);
	root.lookupValue("clusterMinSizeCuda", minPointsForCuda);
	root.lookupValue("masatoQuality", masatoQuality);
    odtc::EuclideanClusteringParameters cluParams(cluTol, minCluSize, maxCluSize, minPointsForCuda);
    cluParams.Quality(masatoQuality);
    dtcParams.ClusteringParameters(cluParams);

    double minObjectIntensity;
    root.lookupValue("minObjectIntensity", minObjectIntensity);
    dtcParams.minObjectIntensity = minObjectIntensity;
    double minLowIntensity;
    root.lookupValue("minLowIntensity", minLowIntensity);
    dtcParams.minLowIntensity = minLowIntensity;
    int minLIObjectNPoints;
    root.lookupValue("minLIObjectNPoints", minLIObjectNPoints);
    dtcParams.minLIObjectNPoints = minLIObjectNPoints;
    std::cerr << "minObjectIntensity = " << dtcParams.minObjectIntensity << std::endl;
    std::cerr << "minLowIntensity = " << dtcParams.minLowIntensity << std::endl;
    std::cerr << "minLIObjectNPoints = " << dtcParams.minLIObjectNPoints << std::endl;
    
    //////////////// POINT CLOUD FILTERING ////////////////
	root.lookupValue("minDistance", tempDouble);
    dtcParams.MinDistance(tempDouble);
	root.lookupValue("maxDistance", tempDouble);
    dtcParams.MaxDistance(tempDouble);
    
	root.lookupValue("minAccumDistancePcl", tempDouble);
    std::cerr << "minAccumDistancePcl = " << tempDouble << std::endl;
    dtcParams.MinAccumDist(tempDouble);

    std::string cloudAccumPolicy;
	root.lookupValue("cloudAccumulationPolicy", cloudAccumPolicy);
    dtcParams.CloudAccumulationPolicy(cloudAccumPolicy);
    std::cerr << "cloudAccumPolicy = " << dtcParams.CloudAccumulationPolicyStr() << std::endl;

    //////////////// POINT CLOUD VOXEL-BASED REDUCTION ////////////////
    root.lookupValue("enableCloudReduction", tempInt);
    dtcParams.EnableCloudReduction(tempInt);

    double leaf_x, leaf_y, leaf_z;
	root.lookupValue("cloudReductionLeaf_x", leaf_x);
	root.lookupValue("cloudReductionLeaf_y", leaf_y);
	root.lookupValue("cloudReductionLeaf_z", leaf_z);
    dtcParams.LeafSize(Eigen::Vector3d(leaf_x,leaf_y,leaf_z));
    
    //////////////// CAMERAS ////////////////
    root.lookupValue("minImgDetectionConfidence", tempDouble);
    dtcParams.MinImgDetectionConfidence(tempDouble);
    
    /////////////// IMAGE PLANE TRACKING ////////////////
    root.lookupValue("enableImgPlaneTracking", tempInt);
    dtcParams.enableImgPlaneTracking = tempInt;

    root.lookupValue("saveClusters", tempInt);
    saveClusters_ = tempInt;

    root.lookupValue("enableTrackingGt", tempInt);
    enableTrackingGt_ = tempInt;
    
    odtc::TrackingParams imgPlaneTrackingParams;
    std::vector<double> imgQdiag;
    auto &imgQdiag_node = root.lookup("imgQdiag");
    for (auto &q : imgQdiag_node) imgQdiag.emplace_back(q);
    imgPlaneTrackingParams.Q.resize(imgQdiag.size(), imgQdiag.size());
    imgPlaneTrackingParams.Q.setZero();
    for (auto idx = 0; idx < imgQdiag.size(); idx++) { imgPlaneTrackingParams.Q(idx, idx) = imgQdiag[idx]; }
    std::cerr << "[ReadDetectionParams] imgQdiag size = " << imgQdiag.size() << std::endl << std::endl;
    std::cerr << "[ReadDetectionParams] imgQ = "<< std::endl << imgPlaneTrackingParams.Q << std::endl;

    // Reading Rdiag from libconfig
    std::vector<double> imgRdiag;
    auto &imgRdiag_node = root.lookup("imgRdiag");
    for (auto &r : imgRdiag_node) imgRdiag.emplace_back(r);
    imgPlaneTrackingParams.R0.resize(imgRdiag.size(), imgRdiag.size());
    imgPlaneTrackingParams.R0.setZero();
    for (auto idx = 0; idx < imgRdiag.size(); idx++) { imgPlaneTrackingParams.R0(idx, idx) = imgRdiag[idx]; }
    std::cerr << "[ReadDetectionParams] imgRdiag size = " << imgRdiag.size() << std::endl;
    std::cerr << "[ReadDetectionParams] imgR0 = "<< std::endl << imgPlaneTrackingParams.R0 << std::endl;

    int imgMaxNumOcclusions = root.lookup("imgMaxNumOcclusions");
    imgPlaneTrackingParams.maxOccl = imgMaxNumOcclusions;
    
    std::map<std::string, odtc::IDAssocParams> imgPlaneAssocParams;
    // Reading weightIoU
    tempDouble = root.lookup("imgWeightIoU");
    imgPlaneAssocParams[odtc::Str(odtc::Metric::IOU)].weight = tempDouble;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::IOU)].maxAbs = 0.99;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::IOU)].sat = 100;

    // Reading weightCentroidDist
    tempDouble = root.lookup("imgWeightCentroidDist");
    imgPlaneAssocParams[odtc::Str(odtc::Metric::DIST)].weight = tempDouble;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::DIST)].maxAbs = 150;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::DIST)].sat = 1500;

    // Reading weightSimilarity
    tempDouble = root.lookup("imgWeightSimilarity");
    imgPlaneAssocParams[odtc::Str(odtc::Metric::SIMILARITY)].weight = tempDouble;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::SIMILARITY)].maxAbs = 10;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::SIMILARITY)].sat = 100;

    // Reading weightSize
    tempDouble = root.lookup("imgWeightSize");
    imgPlaneAssocParams[odtc::Str(odtc::Metric::SIZE)].weight = tempDouble;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::SIZE)].maxAbs = 300;
    imgPlaneAssocParams[odtc::Str(odtc::Metric::SIZE)].sat = 1000;
    imgTracker = odtc::Tracking(imgPlaneTrackingParams,imgPlaneAssocParams);
    
    root.lookupValue("enableRTAnnot", tempInt);
    enableRTAnnot_ = tempInt;
    
    //////////////// MULTITHREADING ////////////////
    root.lookupValue("numThreadsForParallelClustering", tempInt);
    dtcParams.NThreads(tempInt);
    
    //////////////// LOGGING ////////////////
    root.lookupValue("enableMainLog", tempInt);
    dtcParams.EnableMainLog(tempInt);

    root.lookupValue("enableEssentialLog", tempInt);
    dtcParams.EnableEssentialLog(tempInt);

    root.lookupValue("enableLidarWindows", tempInt);
    dtcParams.EnableGraphicalWindows(tempInt);

    //////////////// OTHER ENABLE/DISABLE ////////////////

    root.lookupValue("enableRemoveWaterReturns", tempInt);
    dtcParams.enableRemoveWaterReturns = tempInt;
    
    std::cerr << tc::greenL << "[ReadDetectionParams] Detection parameters was successfully configured!" << tc::none << std::endl;

}

MarineDetector::~MarineDetector() {}
