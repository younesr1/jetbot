#include <jetbot_imu/mpu_6050.hpp>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <array>

using namespace std::literals::chrono_literals;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "jetbot_imu_echo");

    constexpr auto slave_address = 0x68, bus_num = 0;
    ROS_INFO_STREAM("Contacting IMU at address " << slave_address << " on i2c bus " << bus_num);

    Sensors::MPU_6050 imu(bus_num, slave_address);
    if (!imu.SetAccelRange(Sensors::MPU_6050::AccelerometerRange::RANGE_2G))
    {
        ROS_FATAL("Unable to set range of accelerometer");
        return EXIT_FAILURE;
    }

    if (!imu.SetGyroRange(Sensors::MPU_6050::GyroscopeRange::RANGE_250DEG_PER_SEC))
    {
        ROS_FATAL("Unable to set range of gyroscope");
        return EXIT_FAILURE;
    }
    int8_t id = 0;
    if (!imu.ReadID(id))
    {
        ROS_FATAL("Unable to read IMU ID");
        return EXIT_FAILURE;
    }
    ROS_INFO_STREAM("Successfully established communication with IMU! ID = " << id);

    while (ros::ok())
    {
        Eigen::Vector3d theta_dot;
        Eigen::Vector3d accel;
        double temperature;

        bool success = true;
        success &= imu.ReadGyroscope(theta_dot);
        success &= imu.ReadAccelerometer(accel);
        success &= imu.ReadTemperature(temperature);

        if (success)
        {
            ROS_INFO("###");
            ROS_INFO_STREAM("Angular Velocity (rad/s): " << theta_dot);
            ROS_INFO_STREAM("Linear Acceleration (m/s^2): " << accel);
            ROS_INFO_STREAM("Temperature (Celsius): " << temperature);
            ROS_INFO("###");
        }
        else
        {
            ROS_WARN("************ Failed to read from IMU ************");
        }

        std::this_thread::sleep_for(20ms); // 50 Hz
    }

    return EXIT_SUCCESS;
}
