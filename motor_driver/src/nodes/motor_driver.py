#!/usr/bin/python3
from featherwing import FeatherWing
import rospy
from std_msgs import Float64MultiArray

fw = FeatherWing()


def Update(speeds):
    fw.SetLeftMotorSpeed(speeds[0])
    fw.SetRightMotorSpeed(speeds[1])


def consumer():
    rospy.init_node("motor_driver")
    rospy.Subscriber("drivetrain", Float64MultiArray, Update)


def main():
    consumer()


if __name__ == '_main__':
    main()
