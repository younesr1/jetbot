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
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;
    //! LOCATORS: Find good features to track
    cv::goodFeaturesToTrack(bwFrame, tempPoints, _maxFeatures, _rejectionRegion, _minDist);
    // Convert found points to KeyPoints
    for(cv::Point2f const& i : tempPoints) {
        keypoints.push_back(cv::KeyPoint(i, _keyPointDiameter));
    }
    //! DESCRIPTORS: From KeyPoints, generate descriptors
    _orb->compute(frame, keypoints, descriptor);
    //! MATCHER: {table_number = 12, key_size = 6, multi_probe_level = 2} TODO try also 6,12,1
    if(_firstExtraction) {
        _firstExtraction = false;
        _oldDescriptor = descriptor;
        _oldKeypoints = keypoints;
        return featureData{std::vector<std::vector<cv::KeyPoint>>(), std::vector<std::vector<cv::DMatch>>()};
    }
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    // -------------------------------------
    // | {match1a} , {match2a} ... {matchNa} |
    // | {match1b} , {match2b} ... {matchNb} |
    // -------------------------------------
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(descriptor, _oldDescriptor, matches, 2);
    std::vector<std::vector<cv::KeyPoint>> matchedKeypoints;
    for(auto const& i : matches) {
        // ~30% of matches are filtered out for nyc video
        if(i.at(0).distance < (0.75*i.at(1).distance)) {
            matchedKeypoints.push_back(std::vector<cv::KeyPoint>{keypoints.at(i.at(0).queryIdx), _oldKeypoints.at(i.at(0).trainIdx)});
        }
    }
    _oldDescriptor = descriptor;
    _oldKeypoints = keypoints;
    return featureData{matchedKeypoints, matches};
}
