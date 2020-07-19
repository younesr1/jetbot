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
    cv::Mat descriptor;
    //! LOCATORS: Find good features to track
    cv::goodFeaturesToTrack(bwFrame, tempPoints, _maxFeatures, _rejectionRegion, _minDist);
    // Convert found points to KeyPoints
    for(cv::Point2f const& i : tempPoints) {
        locators.push_back(cv::KeyPoint(i, _keyPointDiameter));
    }
    //! DESCRIPTORS: From KeyPoints, generate descriptors
    _orb->compute(frame, locators, descriptor);
    //! MATCHER: {table_number = 12, key_size = 6, multi_probe_level = 2} TODO try also 6,12,1
    std::vector<cv::DMatch> matches;
    if(!_firstExtraction) {
        cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1));
        matcher.match(descriptor, _prvsDescriptor, matches);
    }
    else {
        _firstExtraction = false;
    }
    //! FILTERING: Only keep good matches
    float minDist = 200;
    for(cv::DMatch const& match : matches) {
        if(match.distance < minDist) minDist = match.distance;
    }
    std::vector<cv::DMatch> filteredMatches;
    for(cv::DMatch const& match : matches) {
        if(match.distance <= std::max(minDist, _filteringThreshold)) filteredMatches.push_back(match);
    }
    _prvsDescriptor = descriptor;
    return featureData{locators, descriptor, filteredMatches};
}