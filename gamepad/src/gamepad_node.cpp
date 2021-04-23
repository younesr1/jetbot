#include "gamepad/gamepad.hpp"
#include <ros/ros.h>
#include "gamepad/GamepadState.h"
#include "motor_driver/MotorSpeeds.h"

constexpr auto FREQUENCY = 50, BUFFER_SIZE = 1;

int main(int argc, char **argv)
{
    IO::Gamepad gamepad;

    ros::init(argc, argv, "gamepad");
    ros::NodeHandle nh;

    ros::Publisher motors_publisher = nh.advertise<motor_driver::MotorSpeeds>("/drivetrain", BUFFER_SIZE);
    ros::Publisher gamepad_publisher = nh.advertise<gamepad::GamepadState>("/gamepad", BUFFER_SIZE);

    ros::Rate loop_rate(FREQUENCY);

    while (ros::ok())
    {
        if (!gamepad.Update())
        {
            ROS_WARN("Failed to update gamepad");
        }
        gamepad::GamepadState gamepad_msg;
        gamepad_msg.left_js_x = gamepad.ReadLeftJoystick()[0];
        gamepad_msg.left_js_y = gamepad.ReadLeftJoystick()[1];
        gamepad_msg.right_js_x = gamepad.ReadRightJoystick()[0];
        gamepad_msg.right_js_y = gamepad.ReadRightJoystick()[1];
        gamepad_msg.left_trigger = gamepad.ReadLeftTrigger();
        gamepad_msg.right_trigger = gamepad.ReadRightTrigger();
        gamepad_publisher.publish(gamepad_msg);

        motor_driver::MotorSpeeds speeds_msg;
        const double power_factor = (gamepad.ReadRightTrigger() != 0 ? gamepad.ReadRightTrigger() : -1 * gamepad.ReadLeftTrigger()) / 100.0;
        speeds_msg.left = power_factor * (gamepad.ReadLeftJoystick()[0] > 0 ? 100.0 : 100 + gamepad.ReadLeftJoystick()[0]);
        speeds_msg.right = power_factor * (gamepad.ReadRightJoystick()[0] < 0 ? 100.0 : 100 - gamepad.ReadRightJoystick()[0]);
        motors_publisher.publish(speeds_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}