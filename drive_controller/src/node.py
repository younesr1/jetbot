#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from adafruit_motorkit import MotorKit


class FeatherWing:
    def __init__(self, max_speed):
        self.__drivetrain = MotorKit()
        self.__left = self.__drivetrain.motor1
        self.__right = self.__drivetrain.motor2
        self.__max_speed = max_speed

    # Speed [-max_speed, +max_speed]
    def SetLeftMotorSpeed(self, speed):
        speed = speed / self.__max_speed
        speed = min(1, max(-1, speed))
        self.__left.throttle(speed)

    # Speed [-max_speed, +max_speed]
    def SetRightMotorSpeed(self, speed):
        speed = speed / self.__max_speed
        speed = min(1, max(-1, speed))
        self.__right.throttle(speed)


class Controller:
    def __init__(self, topic, wheel_distance, wheel_radius, max_speed):
        self.topic = topic
        self.wheel_distance = wheel_distance
        self.wheel_radius = wheel_radius
        self.fw = FeatherWing(max_speed)

    def twist_to_pair(self, twist):
        left_speed = (twist.linear.x - twist.angular.z *
                      self.wheel_distance/2)/self.wheel_radius
        right_speed = (twist.linear.x + twist.angular.z *
                       self.wheel_distance/2) / self.wheel_radius
        return (left_speed, right_speed)

    def callback(self, data):
        [left, right] = self.twist_to_pair(data)
        self.fw.SetLeftMotorSpeed(left)
        self.fw.SetRightMotorSpeed(right)
        rospy.loginfo("Setting motor speed")


def main():
    rospy.init_node('drive_controller')

    topic = rospy.get_param("/drive_controller/topic")
    wheel_speed = rospy.get_param("/drive_controller/max_wheel_speed")
    wheel_radius = rospy.get_param("/drive_controller/wheel_radius")
    wheel_seperation = rospy.get_param("/drive_controller/wheel_seperation")
    controller = Controller(topic, wheel_seperation, wheel_radius, wheel_speed)

    rospy.Subscriber(topic, Twist, controller.callback)
    rospy.spin()


if __name__ == '__main__':
    main()
