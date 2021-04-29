#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <string>

namespace Vision {
class RaspiCam {
    public:
    RaspiCam(uint16_t capture_width, uint16_t capture_height, uint16_t display_width, uint16_t display_height, uint8_t fps);
    ~RaspiCam();

    /**
    * @brief Updates internal data structure. Should run in its own thread
    * @return image
    */
    cv::Mat GrabFrame();

    private:
    cv::VideoCapture m_capture;
    static std::string GetPipeline(uint16_t capture_width, uint16_t capture_height, uint16_t display_width, uint16_t display_height, uint8_t fps);
};
}
