#!/usr/bin/python3
import motorkit_robot
import rospy
import time


def main():
    LEFT_TRIM = 0
    RIGHT_TRIM = 0

    robot = motorkit_robot.Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)
    while not rospy.is_shudown():
        for i in range(0, 1, 0.1):
            robot.forward(i, 0.25)
        for i in range(0, 1, 0.1):
            robot.backward(i, 0.25)
        robot.stop()
        time.sleep(1)


if __name__ == '__main__':
    main()
