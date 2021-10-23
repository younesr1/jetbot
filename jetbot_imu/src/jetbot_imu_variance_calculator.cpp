#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <array>
#include <algorithm>
#include <numeric>

#ifdef false

using namespace std::literals::chrono_literals;

constexpr auto SAMPLES = 1000;

// younes todo this node should just subscribe to IMU msgs, not create an imu object

double Variance(std::array<double, SAMPLES> data);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetbot_imu_echo");

    constexpr auto slave_address = 0x68, bus_num = 0;
    ROS_INFO_STREAM("Contacting IMU at address " << slave_address << " on i2c bus " << bus_num);

    Sensors::MPU_6050 imu(bus_num, slave_address, Sensors::MPU_6050::GyroscopeRange::RANGE_250DEG_PER_SEC, Sensors::MPU_6050::AccelerometerRange::RANGE_2G);

    ROS_INFO("IMU reached successfully. Beginning data capture in 3 seconds. Make sure IMU is STILL");
    for (int i = 3; i > 0; i++)
    {
        ROS_INFO_STREAM(i << "...");
        std::this_thread::sleep_for(1s);
    }

    std::array<double, SAMPLES> gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp;

    uint16_t iteration = 0;

    while (iteration < SAMPLES)
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
            gyro_x[iteration] = theta_dot.x();
            gyro_y[iteration] = theta_dot.y();
            gyro_z[iteration] = theta_dot.z();

            accel_x[iteration] = accel.x();
            accel_y[iteration] = accel.y();
            accel_z[iteration] = accel.z();

            temp[iteration] = temperature;

            iteration++;
        }
        else
        {
            ROS_WARN("Failed to read from IMU");
        }

        std::this_thread::sleep_for(20ms); // 50 Hz
    }

    ROS_INFO("++++++++++++++++++++++ RESULT ++++++++++++++++++++++");
    ROS_INFO_STREAM("Gyroscope X Variance: " << Variance(gyro_x));
    ROS_INFO_STREAM("Gyroscope Y Variance: " << Variance(gyro_y));
    ROS_INFO_STREAM("Gyroscope Z Variance: " << Variance(gyro_z));
    ROS_INFO_STREAM("Accelerometer X Variance: " << Variance(accel_x));
    ROS_INFO_STREAM("Accelerometer Y Variance: " << Variance(accel_y));
    ROS_INFO_STREAM("Accelerometer Z Variance: " << Variance(accel_z));
    ROS_INFO_STREAM("Temperature Variance: " << Variance(temp));
    ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++");

    return EXIT_SUCCESS;
}

double Variance(std::array<double, SAMPLES> data)
{
    const auto sz = data.size();
    if (sz < 2)
    {
        return 0.0;
    }

    // Calculate the mean
    const double mean = std::accumulate(data.begin(), data.end(), 0.0) / sz;

    // Now calculate the variance
    auto variance_func = [&mean, &sz](double accumulator, const double &val)
    {
        return accumulator + ((val - mean) * (val - mean) / (sz - 1));
    };

    return std::accumulate(data.begin(), data.end(), 0.0, variance_func);
}
#endif

int main()
{
}