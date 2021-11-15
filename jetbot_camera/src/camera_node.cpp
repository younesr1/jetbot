#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <jetson-inference/depthNet.h>
#include <jetson-utils/gstCamera.h>
#include <camera/image_converter.hpp>
#include <memory>

// node main loop
int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetbot_camera");
    ros::NodeHandle nh;

    /*
     * retrieve parameters
     */
    std::string camera_device, rgb_topic, depth_topic;
    int buffer;
    nh.param<std::string>("/jetbot_camera/device", camera_device, "csi://0");
    nh.param<std::string>("/jetbot_camera/rgb_topic", rgb_topic, "/camera/image/raw/rgb");
    nh.param<std::string>("/jetbot_camera/depth_topic", depth_topic, "/camera/image/raw/depth");
    nh.param<int>("/jetbot_camera/buffer_size", buffer, 10);

    ROS_INFO("opening camera device %s", camera_device.c_str());

    /*
     * open camera device
     */
    videoOptions opt;
    opt.flipMethod = videoOptions::FLIP_ROTATE_180;
    opt.resource = camera_device.c_str();
    auto camera = gstCamera::Create(opt);
    auto depth_net = depthNet::Create(); // younes todo look into different constructors. specify which ML model to use?

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
     * advertise publisher rgb_topics
     */
    ros::Publisher rgb_publisher = nh.advertise<sensor_msgs::Image>(rgb_topic, buffer);
    ros::Publisher depth_publisher = nh.advertise<sensor_msgs::Image>(depth_topic, buffer);

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
        float4 *imgRGBA = NULL;

        // get the latest frame
        constexpr auto TIMEOUT = 1000;
        if (!camera->CaptureRGBA((float **)&imgRGBA, TIMEOUT))
        {
            ROS_ERROR("failed to capture camera frame");
            continue;
        }

        // assure correct image size
        if (!camera_cvt->Resize(camera->GetWidth(), camera->GetHeight(), IMAGE_RGBA32F))
        {
            ROS_ERROR("failed to resize camera image converter");
            continue;
        }

        sensor_msgs::Image ros_rgb_img;
        // populate the message
        if (!camera_cvt->Convert(ros_rgb_img, imageConverter::ROSOutputFormat, imgRGBA))
        {
            ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
            continue;
        }

        rgb_publisher.publish(ros_rgb_image);
        ROS_INFO("published rgb image frame");

        depth_net->Process(imgRGBA, camera->GetWidth(), camera->GetHeight());
        CUDA(cudaDeviceSynchronize());

        sensor_msgs::Image ros_depth_img;
        if (!camera_cvt->Convert(ros_depth_img, imageConverter::ROSOutputFormat, imgRGBA))
        {
            ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
            continue;
        }
        depth_publisher.publish(ros_depth_img);

        ros::spinOnce();
    }

    delete camera;
    return EXIT_SUCCESS;
}
