#!/usr/bin/python3
from featherwing import FeatherWing
import rospy
from motor_driver.msg import MotorSpeeds

fw = FeatherWing()


def Update(speeds):
    fw.SetLeftMotorSpeed(speeds.left)
    fw.SetRightMotorSpeed(speeds.right)


def main():
    rospy.init_node("motor_driver")
    rospy.Subscriber("/drivetrain", MotorSpeeds, Update)
    rospy.spin()


if __name__ == '_main__':
    main()
