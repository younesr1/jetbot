#pragma once
#include <opencv2/opencv.hpp>

class camera {
private:
    std::string _pipeline;
    cv::VideoCapture _cap;
    cv::Mat _img;
    cv::VideoWriter _video;
public:
    camera();
    ~camera();
    void record();
};