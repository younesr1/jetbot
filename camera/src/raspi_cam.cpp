#include "camera/raspi_cam.hpp"
#include "camera/calibration.hpp"

#include <filesystem>
#include <string>

namespace Vision
{
    std::string RaspiCam::GetPipeline(uint16_t capture_width, uint16_t capture_height, uint16_t display_width,
                                      uint16_t display_height, uint8_t fps)
    {
        return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
               std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(fps) +
               "/1 ! nvvidconv flip-method=4 ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
               std::to_string(display_height) +
               ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    RaspiCam::RaspiCam(uint16_t capture_width, uint16_t capture_height, uint16_t display_width, uint16_t display_height,
                       uint8_t fps)
        : m_pipeline(GetPipeline(capture_width, capture_height, display_width, display_height, fps))
    {
    }

    bool RaspiCam::Connect()
    {
        m_capture.open(m_pipeline, cv::CAP_GSTREAMER);
        return m_capture.isOpened();
    }

    bool RaspiCam::GrabFrame(cv::Mat &frame)
    {
        return m_capture.read(frame);
    }

    RaspiCam::~RaspiCam()
    {
        m_capture.release();
    }
} // namespace Vision