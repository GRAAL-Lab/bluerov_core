#include <offline_detection_utils.h>

std::string OfflineDetectionUtils::YOLOLabelToBoatNetLabel(std::string lblIn) {
    if (lblIn.find("sail") != std::string::npos) {
        return "sailboat";
    } else if (lblIn.find("ship") != std::string::npos) {
        return "ship";
    } else if (lblIn.find("speedboat") != std::string::npos) {
        return "speedboat";
    } else if (lblIn.find("small boat") != std::string::npos) {
        return "boat";
    } else if (lblIn.find("boat") != std::string::npos) {
        return "boat";
    } else if (lblIn.find("ferry") != std::string::npos) {
        return "ferry";
    } else if (lblIn.find("buoy") != std::string::npos) {
        return "buoy";
    } else if (lblIn.find("kayak") != std::string::npos) {
        return "kayak";
    } else if (lblIn.find("person") != std::string::npos) {
        return "person";
    } else if (lblIn.find("UFO") != std::string::npos) {
        return "UFO";
    } else if (lblIn.find("fishing boat") != std::string::npos) {
        return "fishing boat";
    } else if (lblIn.find("fishing trawler") != std::string::npos) {
        return "fishing trawler";
    } else if (lblIn.find("yacht") != std::string::npos) {
        return "yacht";
    } else if (lblIn.find("warship") != std::string::npos) {
        return "warship";
    } else {
        return "others";
    }
}

std::map<double, std::vector<odtc::BoundingBox<2>>> OfflineDetectionUtils::AnnotationFileToBoxVector(const std::string fileName) {
    std::map<double, std::vector<odtc::BoundingBox<2>>> detections;

    std::ifstream file(fileName, std::ios::binary);

    bool enableDbg = true;
    
    if (!file.is_open()) {
        //std::__throw_runtime_error("Annotation file not found!");
        std::cerr << tc::redL << "[AnnotationFileToBoxVector] Could not open file " << fileName << tc::none << std::endl; 
        return detections;
    }

    while (file.peek() != EOF) {
        double t;
        file.read(reinterpret_cast<char*>(&t), sizeof(t));
        
        uint32_t floatListLength;
        file.read(reinterpret_cast<char*>(&floatListLength), sizeof(floatListLength));
        
        if (file.eof()) break;

        std::vector<float> floatList(floatListLength);
        file.read(reinterpret_cast<char*>(floatList.data()), floatListLength * sizeof(float));
        
        uint32_t str1Length;
        file.read(reinterpret_cast<char*>(&str1Length), sizeof(str1Length));
        
        std::string str1(str1Length, '\0');
        file.read(&str1[0], str1Length);

        float conf;
        file.read(reinterpret_cast<char*>(&conf), sizeof(conf));

        int intValue;
        file.read(reinterpret_cast<char*>(&intValue), sizeof(intValue));

        auto xC = floatList[0];
        auto yC = floatList[1];
        auto xSize = floatList[2];
        auto ySize = floatList[3];

        auto tKey = t;
        
        auto label = YOLOLabelToBoatNetLabel(str1);
        odtc::BoundingBox<2> detection(Eigen::Vector2d(xC, yC), Eigen::Vector2d(xSize, ySize));
        detection.Confidence(conf);
        detection.Description(label);
        if (detections.find(tKey) == detections.end()) {
            detections[tKey] = std::vector<odtc::BoundingBox<2>>();
        }
       // detection.PrintLong();
        detections[tKey].push_back(detection);
    }

    file.close();
    std::cerr << tc::greenL << "[AnnotationFileToBoxVector] Read ok annotation at " << fileName << tc::none << std::endl; 
    return detections;
}

bool OfflineDetectionUtils::GetBoxes(const std::map<double, std::vector<odtc::BoundingBox<2>>> &m, const double ts, std::vector<odtc::BoundingBox<2>> &result) {
    auto tKey = ts;
    //::cerr << "Searching for key close to " << tKey << std::endl;

    for (const auto& entry : m) {
        if (std::abs(entry.first - tKey) < 1e-1) {
            result = entry.second;
            /*std::cerr << tc::redL;
            for (auto b : result) {
                b.PrintLong();
            }
            std::cerr << tc::none;*/
            //std::cerr << "Found key " << entry.first << " within error margin" << std::endl;
            return true;
        }
    }

    //std::cerr << "No key found within error margin of " << 1e-4 << std::endl;
    return false;
}
