#pragma once
#include <chrono>
#include <opencv2/opencv.hpp>

class camera {
private:
  const std::string _pipeline;
  cv::VideoCapture _cap;
  cv::VideoWriter _video;

public:
  camera(std::string output = "output.avi");
  ~camera();
  void record(const std::chrono::duration<float> duration);
};