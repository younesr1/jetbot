#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

/*
  This program attempts to find a rotation matrix, R, which rotates the
  acceleration vector, a, such that it is parallel to the z-axis.
  Mathematically: cross(Ra, -k^) == [0, 0, 0] && |Ra| == |a|

  This program also computes the covariance matrix of the gyroscope and accelerometer readings.
  Finally, it also produces the median values of each axis reading of both accelerometer and gyroscope readings.

  IMPORTANT: Before running this program, ensure the IMU is still.
*/

struct StratifiedImuData
{
    std::vector<double> accelerometer_x, accelerometer_y, accelerometer_z, gyroscope_x, gyroscope_y, gyroscope_z, temperature;
};

StratifiedImuData ListenForData(const std::string &imu_topic, const std::string &temp_topic, ros::NodeHandle &nh, size_t data_points);

Eigen::Matrix3d GetIMUPose(const Eigen::Vector3d &acceleration);

double CalculateVariance(const std::vector<double> &data);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_rotation_calculator");
    ros::NodeHandle nh;

    std::string imu_topic, temp_topic;
    nh.param<std::string>("/jetbot_imu/imu_topic", imu_topic, "/imu/raw/data");
    nh.param<std::string>("/jetbot_imu/temp_topic", temp_topic, "/imu/raw/temperature");
    int samples;
    nh.param<int>("/jetbot_imu/samples", samples, 100);

    // younes todo get number of samples from param server
    auto data = ListenForData(imu_topic, temp_topic, nh, 100);
    std::sort(data.accelerometer_x.begin(), data.accelerometer_x.end());
    std::sort(data.accelerometer_y.begin(), data.accelerometer_y.end());
    std::sort(data.accelerometer_z.begin(), data.accelerometer_z.end());
    std::sort(data.gyroscope_x.begin(), data.gyroscope_x.end());
    std::sort(data.gyroscope_y.begin(), data.gyroscope_y.end());
    std::sort(data.gyroscope_z.begin(), data.gyroscope_z.end());

    // Compute median values of all data sources
    const double median_accelerometer_x = data.accelerometer_x.at(data.accelerometer_x.size() / 2);
    const double median_accelerometer_y = data.accelerometer_y.at(data.accelerometer_y.size() / 2);
    const double median_accelerometer_z = data.accelerometer_z.at(data.accelerometer_z.size() / 2);
    const double median_gyroscope_x = data.gyroscope_x.at(data.gyroscope_x.size() / 2);
    const double median_gyroscope_y = data.gyroscope_y.at(data.gyroscope_y.size() / 2);
    const double median_gyroscope_z = data.gyroscope_z.at(data.gyroscope_z.size() / 2);

    // Compute variance of all data sources
    const double variance_accelerometer_x = CalculateVariance(data.accelerometer_x);
    const double variance_accelerometer_y = CalculateVariance(data.accelerometer_y);
    const double variance_accelerometer_z = CalculateVariance(data.accelerometer_z);
    const double variance_gyroscope_x = CalculateVariance(data.gyroscope_x);
    const double variance_gyroscope_y = CalculateVariance(data.gyroscope_y);
    const double variance_gyroscope_z = CalculateVariance(data.gyroscope_z);
    const double variance_temperature = CalculateVariance(data.temperature);

    // Compute the orientation of the IMU relative to the robot
    Eigen::Matrix3d rot = GetIMUPose(Eigen::Vector3d(median_accelerometer_x, median_accelerometer_y, median_accelerometer_z));

    ROS_INFO("#################################################################");
    ROS_INFO("CALIBRATION COMPLETE!");
    ROS_INFO("Median and Variance Values");
    ROS_INFO_STREAM("Gyroscope Medians: {" << median_gyroscope_x << ", " << median_gyroscope_y << ", " << median_gyroscope_z << "}");
    ROS_INFO_STREAM("Gyroscope Variances: {" << variance_gyroscope_x << ", " << variance_gyroscope_y << ", " << variance_gyroscope_z << "}");
    ROS_INFO_STREAM("Accelerometer Medians: {" << median_accelerometer_x << ", " << median_accelerometer_y << ", " << median_accelerometer_z << "}");
    ROS_INFO_STREAM("Accelerometer Variances: {" << variance_accelerometer_x << ", " << variance_accelerometer_y << ", " << variance_accelerometer_z << "}");
    ROS_INFO_STREAM("Temperature Variances: " << variance_temperature);

    ROS_INFO("\nIMU Pose Relative to Robot");
    ROS_INFO_STREAM("Computed Rotation Matrix: \n"
                    << rot);

    return EXIT_SUCCESS;
}

StratifiedImuData ListenForData(const std::string &imu_topic, const std::string &temp_topic, ros::NodeHandle &nh, size_t data_points)
{
    StratifiedImuData ret;

    auto imu_callback = [&ret](const sensor_msgs::Imu::ConstPtr &msg) -> void
    {
        ret.accelerometer_x.push_back(msg->linear_acceleration.x);
        ret.accelerometer_y.push_back(msg->linear_acceleration.y);
        ret.accelerometer_z.push_back(msg->linear_acceleration.z);
        ret.gyroscope_x.push_back(msg->angular_velocity.x);
        ret.gyroscope_y.push_back(msg->angular_velocity.y);
        ret.gyroscope_z.push_back(msg->angular_velocity.z);
    };

    auto temp_callback = [&ret](const sensor_msgs::Temperature::ConstPtr &msg) -> void
    {
        ret.temperature.push_back(msg->temperature);
    };

    auto imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, imu_callback);
    auto temp_sub = nh.subscribe<sensor_msgs::Temperature>(temp_topic, 100, temp_callback);

    while (ret.accelerometer_x.size() < data_points || ret.temperature.size() < data_points)
    {
        ros::spinOnce();
    }
    return ret;
}

Eigen::Matrix3d GetIMUPose(const Eigen::Vector3d &acceleration)
{
    Eigen::Vector3d rotated_accel(0, 0, -acceleration.norm());
    Eigen::Matrix3d rot = Eigen::Quaterniond().setFromTwoVectors(acceleration, rotated_accel).toRotationMatrix();

    if (!rotated_accel.isApprox(rot * acceleration))
    {
        ROS_ERROR("Unable to compute required transformation from");
        return Eigen::Matrix3d::Zero();
    }
    return rot;
}

double CalculateVariance(const std::vector<double> &data)
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
