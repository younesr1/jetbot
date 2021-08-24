#pragma once
#include <opencv2/opencv.hpp>
#include <array>

/*
[[-0.35044096  0.1601711   0.00110632  0.00121912 -0.03975757]]
[[466.43456808   0.         463.83513373]
 [  0.         461.41832754 369.14728741]
 [  0.           0.           1.        ]]
 */
namespace Vision
{
    namespace Calibration
    {
        constexpr double k1 = -0.35044096, k2 = 0.1601711, p1 = 0.00110632, p2 = 0.00121912, k3 = -0.03975757;
        constexpr double fx = 466.43456808, fy = 461.41832754, cx = 463.83513373, cy = 369.14728741;

        std::array<double, 5> intrinsics = {k1, k2, p1, p2, k3};
        const cv::Mat Intrinsics(1, 5, CV_64FC1, intrinsics.data());

        std::array<double, 9> distortions = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        const cv::Mat Distortions(3, 3, CV_64FC1, distortions.data());
    }
}