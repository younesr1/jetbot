#include "featureExtractor.h"
#include "opencv2/imgproc.hpp"

featureExtractor::featureExtractor() : _orb(cv::ORB::create()), _firstExtraction(true) {}

featureExtractor::featureData featureExtractor::extractFeatures(const cv::Mat &frame) {
    // Black and white frame needed for finding good features to track
    cv::Mat bwFrame;
    cv::cvtColor(frame, bwFrame, cv::COLOR_BGR2GRAY);
    // Temporary vector for holding (x,y) locations of good features found
    std::vector<cv::Point2f> tempPoints;
    // Vectors for holding all the keypoint-locators and descriptors of the good features found
    std::vector<cv::KeyPoint> locators;
    cv::Mat descriptors;
    // Find good features to track
    cv::goodFeaturesToTrack(bwFrame, tempPoints, _maxFeatures, _rejectionRegion, _minDist);
    // Convert found points to KeyPoints
    for(cv::Point2f const& i : tempPoints) {
        locators.push_back(cv::KeyPoint(i, _keyPointDiameter));
    }
    // From KeyPoints, generate descriptors
    _orb->compute(frame, locators, descriptors);
    _prvsFrame = frame;
    return featureData{locators, descriptors, 42};
}