#include <jetbot_imu/mpu_6050.hpp>
#include <unordered_map>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <ros/ros.h>

Sensors::MPU_6050::AccelerometerRange StringToAccelRange(const std::string &range);

Sensors::MPU_6050::GyroscopeRange StringToGyroRange(const std::string &range);

// node main loop younes todo add launch file
int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetbot_imu");
    ros::NodeHandle nh;

    /*
	 * retrieve parameters
	 */
    int bus_num, slave_address, buffer, freq;
    std::string imu_topic, temp_topic, gyro_range, accel_range;
    nh.param<int>("/jetbot_imu/bus_number", bus_num, 0);
    nh.param<int>("/jetbot_imu/frequency", freq, 50);
    nh.param<int>("/jetbot_imu/imu_slave_address", slave_address, 0x68);
    nh.param<int>("/jetbot_imu/buffer_size", buffer, 10);
    nh.param<std::string>("/jetbot_imu/imu_topic", imu_topic, "/imu/raw/data");
    nh.param<std::string>("/jetbot_imu/temp_topic", temp_topic, "/imu/raw/temperature");
    nh.param<std::string>("/jetbot_imu/gyro_range", gyro_range, "250");
    nh.param<std::string>("/jetbot_imu/accel_range", accel_range, "2G");

    ROS_INFO_STREAM("Contacting IMU at address " << slave_address << " on i2c bus " << bus_num);

    Sensors::MPU_6050 imu(bus_num, slave_address, StringToGyroRange(gyro_range), StringToAccelRange(accel_range));

    /*
	 * advertise publisher topics
	 */
    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>(imu_topic, buffer);
    ros::Publisher temp_publisher = nh.advertise<sensor_msgs::Temperature>(temp_topic, buffer);
    ros::Rate loop_rate(freq);

    /*
	 * start publishing 
	 */
    while (ros::ok())
    {
        Eigen::Vector3d theta_dot;
        Eigen::Vector3d accel;
        double temperature;

        bool success = true;
        success &= imu.ReadGyroscope(theta_dot);
        success &= imu.ReadAccelerometer(accel);
        success &= imu.ReadTemperature(temperature);

        Eigen::Matrix<double, 9, 1> gyro_covariance = imu.GetGyroCovariance();
        Eigen::Matrix<double, 9, 1> accel_covariance = imu.GetAccelCovariance();
        double temp_variance = imu.GetTempVariance();

        if (success)
        {
            sensor_msgs::Imu imu_msg;

            imu_msg.angular_velocity.x = theta_dot.x();
            imu_msg.angular_velocity.y = theta_dot.y();
            imu_msg.angular_velocity.z = theta_dot.z();

            imu_msg.linear_acceleration.x = accel.x();
            imu_msg.linear_acceleration.y = accel.y();
            imu_msg.linear_acceleration.z = accel.z();

            for (int i = 0; i < 9; i++)
            {
                imu_msg.angular_velocity_covariance[i] = gyro_covariance[i];
                imu_msg.linear_acceleration_covariance[i] = accel_covariance[i];
            }

            // imu does not provide oritentation estimate, so set first index of covariance to -1 as required
            imu_msg.orientation_covariance[0] = -1;

            imu_publisher.publish(imu_msg);

            sensor_msgs::Temperature temp_msg;
            temp_msg.temperature = temperature;
            temp_msg.variance = temp_variance;
            temp_publisher.publish(temp_msg);
        }
        else
        {
            ROS_WARN("Failed to read from IMU");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return EXIT_SUCCESS;
}

Sensors::MPU_6050::AccelerometerRange StringToAccelRange(const std::string &range)
{
    static const std::unordered_map<std::string, Sensors::MPU_6050::AccelerometerRange> map = {
        {"2G", Sensors::MPU_6050::AccelerometerRange::RANGE_2G},
        {"4G", Sensors::MPU_6050::AccelerometerRange::RANGE_4G},
        {"8G", Sensors::MPU_6050::AccelerometerRange::RANGE_8G},
        {"16G", Sensors::MPU_6050::AccelerometerRange::RANGE_16G},
    };
    return map.at(range);
}

Sensors::MPU_6050::GyroscopeRange StringToGyroRange(const std::string &range)
{
    static const std::unordered_map<std::string, Sensors::MPU_6050::GyroscopeRange> map = {
        {"250", Sensors::MPU_6050::GyroscopeRange::RANGE_250DEG_PER_SEC},
        {"500", Sensors::MPU_6050::GyroscopeRange::RANGE_500DEG_PER_SEC},
        {"1000", Sensors::MPU_6050::GyroscopeRange::RANGE_1000DEG_PER_SEC},
        {"2000", Sensors::MPU_6050::GyroscopeRange::RANGE_2000DEG_PER_SEC},
    };
    return map.at(range);
}