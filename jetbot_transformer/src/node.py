#!/usr/bin/python3
import rospy
import tf2_ros as tf2
import tf2_geometry_msgs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


def main():
    rospy.init_node("transformer")

    imu_topic = rospy.get_param("/jetbot_transformer/imu_topic")
    transformed_imu_topic = rospy.get_param("/jetbot_transformer/transformed_imu_topic")

    transformed_imu_pub = rospy.Publisher(transformed_imu_topic, Imu, queue_size=5)

    tf_buffer = tf2.Buffer()
    _ = tf2.TransformListener(tf_buffer)

    def transformed_imu_callback(data: Imu) -> None:
        transform = tf_buffer.lookup_transform('JetBot__chassis', 'JetBot__imu_link', rospy.Time(0))
        
        transformed_imu_data = Imu()
        transformed_imu_data.header.frame_id = "/JetBot__chassis"
        transformed_imu_data.header.stamp = data.header.stamp
        transformed_imu_data.header.seq = data.header.seq

        angular_velocity = Vector3Stamped()
        angular_velocity.header = data.header
        angular_velocity.vector = data.angular_velocity

        linear_acceleration = Vector3Stamped()
        linear_acceleration.header = data.header
        linear_acceleration.vector = data.linear_acceleration

        transformed_imu_data.angular_velocity = tf2_geometry_msgs.do_transform_vector3(angular_velocity, transform).vector
        transformed_imu_data.linear_acceleration = tf2_geometry_msgs.do_transform_vector3(linear_acceleration, transform).vector

        transformed_imu_pub.publish(transformed_imu_data)


    rospy.Subscriber(imu_topic, Imu, transformed_imu_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
