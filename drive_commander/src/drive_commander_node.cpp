#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <limits>

constexpr auto BUFFER_SIZE = 10;
constexpr auto SUBSCRIBE_TOPIC = "/gamepad/data", PUBLISH_TOPIC = "/cmd_vel";
constexpr auto MAX_LINEAR = 10, MAX_ANGULAR = 30;

class Commander
{
private:
    ros::NodeHandle m_nh;
    ros::Publisher m_twist_pub;
    ros::Subscriber m_joy_sub;

    int m_buffer_size;
    std::string m_joy_topic, m_twist_topic;
    double m_max_linear, m_max_angular;

public:
    Commander();
    ~Commander() = default;
    void advertise();
    void cb(const sensor_msgs::Joy::ConstPtr &msg);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_commander");
    Commander cmder;
    cmder.advertise();
    ros::spin();
    return 0;
}

Commander::Commander()
{
    m_nh.getParam("buffer_size", m_buffer_size);
    m_nh.getParam("joy_topic", m_joy_topic);
    m_nh.getParam("twist_topic", m_twist_topic);
    m_nh.getParam("max_linear_speed", m_max_linear);
    m_nh.getParam("max_angular_speed", m_max_angular);
}

void Commander::advertise()
{
    ros::Publisher twist_pub = m_nh.advertise<geometry_msgs::Twist>(m_twist_topic, m_buffer_size);
    ros::Subscriber joy_sub = m_nh.subscribe(m_joy_topic, m_buffer_size, &Commander::cb, this);
}

void Commander::cb(const sensor_msgs::Joy::ConstPtr &msg)
{
    // y-component of left js
    float linear = msg->axes[1] / std::numeric_limits<int16_t>::max() * m_max_linear;
    // x-component of right js
    float angular = msg->axes[2] / std::numeric_limits<int16_t>::max() * m_max_angular;
    // left bumper
    bool deadman = msg->buttons[0] > 1e-5;

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = linear * deadman;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = angular * deadman;
    m_twist_pub.publish(twist_msg);
}