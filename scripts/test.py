#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import time

def joy_cb(data):
    joy = data
    print(joy.buttons[1])

if __name__ == '__main__':
    rospy.init_node('test')

    rate = rospy.Rate(10)

    joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)
    rospy.spin()