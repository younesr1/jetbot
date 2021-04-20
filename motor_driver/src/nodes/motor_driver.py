#!/usr/bin/python3
from featherwing import FeatherWing
import rospy
from std_msgs import Float64

fw = FeatherWing()


def UpdateLeft(speed):
    fw.SetLeftMotorSpeed(speed)


def UpdateRight(speed):
    fw.SetRightMotorSpeed(speed)


def consumer():
    rospy.init_node("motor_driver")
    rospy.Subscriber("/drivetrain/left", Float64, UpdateLeft)
    rospy.Subscriber("/drivetrain/right", Float64, UpdateRight)


def main():
    consumer()


if __name__ == '_main__':
    main()
