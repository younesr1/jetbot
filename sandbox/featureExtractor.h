#pragma once
#include <opencv2/core/mat.hpp>
#include "opencv2/features2d.hpp"
#include <vector>

class featureExtractor {
public:
    struct featureData {
        // this is returned from the 0.75 ratio test
        std::vector<std::vector<cv::KeyPoint>> matchedKeypoints;
        float retention_ratio;
    };
    featureExtractor();
    // ~featureExtractor();
    featureData extractFeatures(const cv::Mat &frame);
private:
    bool _firstExtraction;
    const uint16_t _maxFeatures = 3000;
    const uint8_t _minDist = 3, _keyPointDiameter = 20;
    // reject the bottom 1% of detected features
    const float _rejectionRegion = 0.01f, _filteringThreshold = 0.02f, _ratio = 0.60f;
    cv::Mat _oldDescriptor;
    std::vector<cv::KeyPoint> _oldKeypoints;
    cv::Ptr<cv::ORB> _orb;
    std::vector<cv::KeyPoint> _prvsKeypoints;
};