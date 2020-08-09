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
    return featureData{keypoints};
    #if false
    //! MATCHER: {table_number = 12, key_size = 6, multi_probe_level = 2} TODO try also 6,12,1
    std::vector<std::vector<cv::Point2i>> matchedPoints;
    std::vector<std::vector<cv::DMatch>> matches;
    if(!_firstExtraction) {
        cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING);
        matcher.knnMatch(_prvsDescriptor, descriptor, matches, 2);
        for(auto const& i : matches) {
          if(i.at(0).distance < 0.75*i.at(1).distance)
            matchedPoints.push_back(std::vector<cv::Point2i>{keypoints[i.at(0).queryIdx].pt, _prvsKeypoints[i.at(0).queryIdx].pt});
        }

    }
    else {
        _firstExtraction = false;
    }
    /*
    # filter
    if len(ret) > 0:
      ret = np.array(ret)
      model, inliers = ransac((ret[:, 0], ret[:, 1]),
                              FundamentalMatrixTransform,
                              min_samples=8,
                              residual_threshold=1,
                              max_trials=100)
      ret = ret[inliers]
    //! FILTERING: Only keep good matches
    _prvsDescriptor = descriptor;*/
    _prvsDescriptor = descriptor;
    _prvsKeypoints = keypoints;
    return featureData{matchedPoints};
    #endif
}