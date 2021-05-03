#include "camera/raspi_cam.hpp"

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
        : m_capture(GetPipeline(capture_width, capture_height, display_width, display_height, fps), cv::CAP_GSTREAMER)
    {
        if (!m_capture.isOpened())
        {
            throw std::filesystem::filesystem_error("Camera not detected", std::string(), std::error_code());
        }
    }

    cv::Mat RaspiCam::GrabFrame()
    {
        cv::Mat frame;
        m_capture.read(frame);
        // younes todo undistort
        return frame;
    }
} // namespace Vision