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
    nh.param<std::string>("/jetbot_camera/device", camera_device, "csi://0");
    nh.param<std::string>("/jetbot_camera/topic", topic, "/camera/image/raw");
    nh.param<int>("/jetbot_camera/buffer_size", buffer, 10);

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
