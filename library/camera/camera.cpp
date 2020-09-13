#include "camera.h"
#include "config.h"
#include "featureExtractor.h"

camera::camera(std::string output)
    : _pipeline(
          "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" +
          std::to_string(CONFIG::CAMERA::WIDTH) + ", height=(int)" +
          std::to_string(CONFIG::CAMERA::HEIGHT) +
          ", format=(string)NV12, framerate=(fraction)" +
          std::to_string(CONFIG::CAMERA::FPS) + "/1 ! nvvidconv flip-method=" +
          std::to_string(CONFIG::CAMERA::FLIP) + " ! video/x-raw, width=(int)" +
          std::to_string(CONFIG::CAMERA::WIDTH) + ", height=(int)" +
          std::to_string(CONFIG::CAMERA::HEIGHT) +
          ", format=(string)BGRx ! videoconvert ! video/x-raw, "
          "format=(string)BGR ! appsink"),
      _cap(_pipeline, cv::CAP_GSTREAMER),
      _video(output, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
             CONFIG::CAMERA::FPS,
             cv::Size(CONFIG::CAMERA::WIDTH, CONFIG::CAMERA::HEIGHT)) {
  if (!_cap.isOpened()) {
    std::cout << "Failed to open camera" << std::endl;
    exit(1);
  }
}

camera::~camera() {
  _cap.release();
  _video.release();
}

void camera::record(const std::chrono::duration<float> duration) {
  auto begin(std::chrono::steady_clock::now());
  auto end(std::chrono::steady_clock::now());
  featureExtractor extractor;
  while (end - begin < duration) {
    end = std::chrono::steady_clock::now();
    cv::Mat frame;
    if (!_cap.read(frame)) {
      std::cout << "Capture read error" << std::endl;
      break;
    }
    auto ret = extractor.extractFeatures(frame);
    for (auto const &i : ret.matchedKeypoints) {
      if (!i.empty()) {
        cv::circle(frame, i.at(0).pt, CONFIG::SLAM::FEATURECIRCLESIZE, {0,255,0});
        cv::line(frame, i.at(0).pt, i.at(1).pt, {0,0,255});
      }
    }
    _video.write(frame);
  }
  std::cout << "Ending camera recording." << std::endl;
}