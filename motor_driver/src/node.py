#!/usr/bin/env python
from jetbot.motor_driver.src.featherwing import FeatherWing
import rospy
from geometry_msgs.msg import Twist

topic = "/cmd_vel"

fw = FeatherWing()

def callback(data):
    pass
    
def listener():
    rospy.init_node('listener')
    rospy.Subscriber(topic, Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()