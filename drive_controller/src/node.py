#!/usr/bin/python2.7
import rospy
from geometry_msgs.msg import Twist
from motorkit_robot import Robot


def main():
    rospy.init_node('drive_controller')

    topic = rospy.get_param("/drive_controller/topic")
    left_trim = rospy.get_param("/drive_controller/left_trim")
    right_trim = rospy.get_param("/drive_controller/right_trim")

    robot = Robot(left_trim=left_trim, right_trim=right_trim)
    func = lambda twist : robot.linear(twist.linear.x) if twist.linear.x != 0 else robot.angular(twist.angular.z) 

    rospy.Subscriber(topic, Twist, func)
    rospy.spin()


if __name__ == '__main__':
    main()
