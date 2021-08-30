#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from motorkit_robot import Robot


def main():
    rospy.init_node('drive_controller')

    topic = rospy.get_param("/drive_controller/topic")
    left_trim = rospy.get_param("/drive_controller/left_trim")
    right_trim = rospy.get_param("/drive_controller/right_trim")

    robot = Robot(left_trim=left_trim, right_trim=right_trim)

    # younes todo the steer function cant handle negative speeds, might behave weirdly
    rospy.Subscriber(topic, Twist, lambda twist: robot.steer(twist.linear.x, -twist.angular.z)
                     )
    rospy.spin()


if __name__ == '__main__':
    main()
