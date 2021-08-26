#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera/raspi_cam.hpp>
#include <camera/calibration.hpp>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;

    int cap_width, cap_height, disp_width, disp_height, buffer_size;
    double frequency;
    std::string raw_img_topic, undistorted_img_topic;

    bool parameter_validity = nh.getParam("/camera/raw_img_topic", raw_img_topic);
    parameter_validity &= nh.getParam("/camera/undistorted_img_topic", undistorted_img_topic);
    parameter_validity &= nh.getParam("/camera/buffer_size", buffer_size);
    parameter_validity &= nh.getParam("/camera/frequency", frequency);
    parameter_validity &= nh.getParam("/camera/cap_width", cap_width);
    parameter_validity &= nh.getParam("/camera/cap_height", cap_height);
    parameter_validity &= nh.getParam("/camera/disp_width", disp_width);
    parameter_validity &= nh.getParam("/camera/disp_height", disp_height);

    if (!parameter_validity)
    {
        ROS_FATAL("Could not retrieve required parameters");
        return EXIT_FAILURE;
    }

    image_transport::ImageTransport it(nh);
    image_transport::Publisher raw_pub = it.advertise(raw_img_topic, buffer_size);
    image_transport::Publisher undistorted_pub = it.advertise(undistorted_img_topic, buffer_size);

    Vision::RaspiCam cam(cap_width, cap_height, disp_width, disp_height, frequency);

    while (!cam.Connect() && ros::ok())
    {
        ROS_WARN("Unable to connect to Raspi Camera. Trying again in 2 seconds");
        std::this_thread::sleep_for(2s);
    }
    ROS_INFO("Found Raspi Camera");

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        cv::Mat img;
        if (cam.GrabFrame(img))
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            raw_pub.publish(msg);

            cv::Mat undistorted;
            cv::undistort(img, undistorted, Vision::Calibration::Intrinsics, Vision::Calibration::Distortions);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted).toImageMsg();
            undistorted_pub.publish(msg);
            ROS_INFO("Published Image");
        }
        else
        {
            ROS_WARN("Error retrieving image");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}