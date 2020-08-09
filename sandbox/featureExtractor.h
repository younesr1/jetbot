#pragma once
#include <opencv2/core/mat.hpp>
#include "opencv2/features2d.hpp"
#include <vector>

class featureExtractor {
public:
    struct featureData {
        std::vector<cv::KeyPoint> keypoints;
    };
    featureExtractor();
    // ~featureExtractor();
    featureData extractFeatures(const cv::Mat &frame);
private:
    bool _firstExtraction;
    const uint16_t _maxFeatures = 3000;
    const uint8_t _minDist = 3, _keyPointDiameter = 20;
    // reject the bottom 1% of detected features
    const float _rejectionRegion = 0.01f, _filteringThreshold = 0.02f;
    cv::Mat /*_prvsFrame, */_prvsDescriptor;
    cv::Ptr<cv::ORB> _orb;
    std::vector<cv::KeyPoint> _prvsKeypoints;
};