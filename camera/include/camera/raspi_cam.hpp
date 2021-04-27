#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>

namespace Vision {
class RaspiCam {
    public:
    RaspiCam(uint16_t width, uint16_t height, uint8_t fps);
    ~RaspiCam();

    /**
    * @brief Updates internal data structure. Should run in its own thread
    * @return image
    */
    cv::Mat GrabFrame();

    private:
    cv::VideoCapture m_capture;
    static auto GetPipeline(uint16_t width, uint16_t height, uint8_t fps);
};
}
