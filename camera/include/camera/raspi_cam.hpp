#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <string>

namespace Vision
{
    class RaspiCam
    {
    public:
        RaspiCam(uint16_t capture_width, uint16_t capture_height, uint16_t display_width, uint16_t display_height, uint8_t fps);
        ~RaspiCam();

        /**
        * @brief grabs an image
        * @return success
        */
        bool GrabFrame(cv::Mat& frame);

        /**
        * @brief Connects the camera
        * @return success
        */
        bool Connect();

    private:
        std::string m_pipeline;
        cv::VideoCapture m_capture;
        static std::string GetPipeline(uint16_t capture_width, uint16_t capture_height, uint16_t display_width, uint16_t display_height, uint8_t fps);
    };
}
