#include "gamepad/gamepad.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamepad");
    ros::NodeHandle nh;

    std::string topic, path;
    int frequency, buffer_size;

    nh.getParam("topic", topic);
    nh.getParam("path", path);
    nh.getParam("frequency", frequency);
    nh.getParam("buffer_size", buffer_size);

    ros::Rate loop_rate(frequency);
    ros::Publisher pub = nh.advertise<sensor_msgs::Joy>(topic, buffer_size);

    IO::Gamepad gamepad(path);
    while (!gamepad.Connect() && ros::ok())
    {
        ROS_WARN("Unable to find PS4 controller. Trying again in 2 seconds");
        std::this_thread::sleep_for(2s);
    }

    while (ros::ok())
    {
        if (!gamepad.Update())
        {
            ROS_WARN("Failed to update gamepad");
        }
        auto state = gamepad.Read();
        sensor_msgs::Joy msg;
        msg.axes.clear();
        msg.buttons.clear();
        msg.axes = {state.left_js[0], state.left_js[1], state.right_js[0], state.right_js[1], state.left_trigger, state.right_trigger};
        msg.buttons = {state.left_bumper, state.right_bumper, state.triangle_button, state.circle_button, state.x_button, state.square_button};
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}