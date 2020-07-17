#pragma once
#include <opencv2/core/mat.hpp>
#include "opencv2/features2d.hpp"
#include <vector>

class featureExtractor {
public:
    struct featureData {
        std::vector<cv::KeyPoint> locators;
        cv::Mat decriptors;
        int matcher;
    };
    featureExtractor();
    // ~featureExtractor();
    featureData extractFeatures(const cv::Mat &frame);
private:
    bool _firstExtraction;
    const uint16_t _maxFeatures = 1000;
    const uint8_t _minDist = 10, _keyPointDiameter = 20;
    // reject the bottom 1% of detected features
    const float _rejectionRegion = 0.01f;
    cv::Mat _prvsFrame;
    cv::Ptr<cv::ORB> _orb;
};