#include "camera/raspi_cam.hpp"
#include <string>
#include <filesystem>

// younes todo run a formatter on everything
namespace Vision {
    /*gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
   'video/x-raw(memory:NVMM),width=3280, height=2464, framerate=21/1, format=NV12' ! \
   nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=720' ! \
   nvvidconv ! nvegltransform ! nveglglessink -e*/
    std::string RaspiCam::GetPipeline(uint16_t capture_width, uint16_t capture_height, uint16_t display_width, uint16_t display_height, uint8_t fps) {
        return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" +
          std::to_string(capture_width) + ", height=(int)" +
          std::to_string(capture_height) +
          ", format=(string)NV12, framerate=(fraction)" +
          std::to_string(fps) + "/1 ! nvvidconv flip-method=4 ! video/x-raw, width=(int)" +
          std::to_string(display_width) + ", height=(int)" +
          std::to_string(display_height) +
          ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    RaspiCam::RaspiCam(uint16_t capture_width, uint16_t capture_height, uint16_t display_width, uint16_t display_height, uint8_t fps) : m_capture(GetPipeline(capture_width, capture_height, display_width, display_height, fps), cv::CAP_GSTREAMER) {
        if(!m_capture.isOpened()) {
            throw std::filesystem::filesystem_error("Camera not detected", std::string(), std::error_code());
        }
    }

    cv::Mat RaspiCam::GrabFrame() {
        cv::Mat frame;
        m_capture.read(frame);
        // todo undistort
        return frame;
    }
}