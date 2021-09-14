#!/usr/bin/python3
import motorkit_robot
import rospy
import time
import numpy as np

def main():
    LEFT_TRIM = 0
    RIGHT_TRIM = 0

    robot = motorkit_robot.Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)
    while not rospy.is_shutdown():
        for i in np.linspace(-1, 1, 25):
            robot.linear(i)
            time.sleep(0.25)
        robot.stop()
        time.sleep(1)


if __name__ == '__main__':
    main()
