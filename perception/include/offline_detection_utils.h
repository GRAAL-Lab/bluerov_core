#ifndef MARINE_DETECTION_OFFLINE_TEST_UTILS_H
#define MARINE_DETECTION_OFFLINE_TEST_UTILS_H

#include <vector>
#include <odtc/BoundingBox.h>
#include <odtc/Camera.h>

class OfflineDetectionUtils {
public:

public:
static std::string YOLOLabelToBoatNetLabel(std::string lblIn);
static std::map<double, std::vector<odtc::BoundingBox<2>>> AnnotationFileToBoxVector(const std::string filePath);
static bool GetBoxes(const std::map<double, std::vector<odtc::BoundingBox<2>>> &m, const double ts, std::vector<odtc::BoundingBox<2>> &result);

};

#endif