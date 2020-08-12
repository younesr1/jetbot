#pragma once
#include <opencv2/core/mat.hpp>
#include "opencv2/features2d.hpp"
#include <vector>

class featureExtractor {
public:
    struct featureData {
        // this is returned from the 0.75 ratio test
        std::vector<std::vector<cv::KeyPoint>> matchedKeypoints;
    };
    featureExtractor();
    // ~featureExtractor();
    featureData extractFeatures(const cv::Mat &frame);
private:
    bool _firstExtraction;
    cv::Mat _oldDescriptor;
    std::vector<cv::KeyPoint> _oldKeypoints;
    cv::Ptr<cv::ORB> _orb;
    std::vector<cv::KeyPoint> _prvsKeypoints;
};