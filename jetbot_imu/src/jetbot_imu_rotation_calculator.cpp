#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>
#include <sensor_msgs/Imu.h>

/*
  This program attempts to find a rotation matrix, R, which rotates the
  acceleration vector, a, such that it is parallel to the z-axis.
  Mathematically: cross(Ra, -k^) == [0, 0, 0]
*/

std::vector<Eigen::Vector3d> ListenForData(const std::string &topic, ros::NodeHandle &nh, size_t data_points);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_rotation_calculator");
    ros::NodeHandle nh;

    std::string imu_topic;
    nh.param<std::string>("/jetbot_imu/imu_topic", imu_topic, "/imu/raw/data");

    auto data = ListenForData(imu_topic, nh, 100);

    std::sort(data.begin(), data.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
              { return a.norm() > b.norm(); });
    Eigen::Vector3d accel = data.at(data.size() / 2);
    Eigen::Vector3d rotated_accel(0, 0, -accel.norm());

    Eigen::Matrix3d rot = Eigen::Quaterniond().setFromTwoVectors(accel, rotated_accel).toRotationMatrix();

    if (!rotated_accel.isApprox(rot * accel))
    {
        ROS_ERROR("Unable to compute required transformation from");
        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("Computed Rotation Matrix: \n"
                    << rot);
    ROS_INFO_STREAM("Compute Euler Angles (Degrees): \n"
                    << rot.eulerAngles(0, 1, 2) * 180 / M_PI);

    return EXIT_SUCCESS;
}

std::vector<Eigen::Vector3d> ListenForData(const std::string &topic, ros::NodeHandle &nh, size_t data_points)
{
    std::vector<Eigen::Vector3d> ret(data_points);
    assert(ret.size() == 0);
    auto callback = [&ret](const sensor_msgs::Imu::ConstPtr &msg) -> void
    { ret.push_back(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)); };
    auto sub = nh.subscribe<sensor_msgs::Imu>(topic, 100, callback);

    while (ret.size() < data_points)
    {
        ros::spinOnce();
    }
    return ret;
}