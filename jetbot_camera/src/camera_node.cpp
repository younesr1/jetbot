/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <memory>
#include <jetson-utils/gstCamera.h>
#include "camera/image_converter.hpp"

bool GrabImage(gstCamera *camera, std::unique_ptr<imageConverter> &camera_cvt, sensor_msgs::Image &img)
{
    float4 *imgRGBA = NULL;

    // get the latest frame
    constexpr auto timeout = 1000; //ms
    if (!camera->CaptureRGBA((float **)&imgRGBA, timeout))
    {
        ROS_ERROR("failed to capture camera frame");
        return false;
    }

    // assure correct image size
    if (!camera_cvt->Resize(camera->GetWidth(), camera->GetHeight(), IMAGE_RGBA32F))
    {
        ROS_ERROR("failed to resize camera image converter");
        return false;
    }

    // populate the message
    if (!camera_cvt->Convert(img, imageConverter::ROSOutputFormat, imgRGBA))
    {
        ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
        return false;
    }
    return true;
}

// node main loop
int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetbot_camera");
    ros::NodeHandle nh;

    /*
	 * retrieve parameters
	 */
    std::string camera_device, topic;
    int buffer;
    nh.param<std::string>("device", camera_device, "csi://0");
    nh.param<std::string>("topic", topic, "/camera/image/raw");
    nh.param<int>("buffer_size", buffer, 10);

    ROS_INFO("opening camera device %s", camera_device.c_str());

    /*
	 * open camera device
	 */
    videoOptions opt;
    opt.flipMethod = videoOptions::FLIP_ROTATE_180;
    opt.resource = camera_device.c_str();
    auto camera = gstCamera::Create(opt);

    if (!camera)
    {
        ROS_ERROR("failed to open camera device %s", camera_device.c_str());
        return EXIT_FAILURE;
    }

    /*
	 * create image converter
	 */
    auto camera_cvt = std::make_unique<imageConverter>();

    if (!camera_cvt)
    {
        ROS_ERROR("failed to create imageConverter");
        return EXIT_FAILURE;
    }

    /*
	 * advertise publisher topics
	 */
    ros::Publisher camera_publisher = nh.advertise<sensor_msgs::Image>(topic, buffer);
    /*
	 * start the camera streaming
	 */
    if (!camera->Open())
    {
        ROS_ERROR("failed to start camera streaming");
        return EXIT_FAILURE;
    }

    /*
	 * start publishing video frames
	 */
    while (ros::ok())
    {
        sensor_msgs::Image msg;
        if (GrabImage(camera, camera_cvt, msg))
        {
            camera_publisher.publish(msg);
            ROS_INFO("published camera frame");
        }

        ros::spinOnce();
    }

    delete camera;
    return EXIT_SUCCESS;
}
