#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
from threading import Thread
from sensor_msgs.msg import Joy


def joy_cb(data):
    global joy
    joy = data

def handler(cf):
    cf.setParam("commander/enHighLevel", 1)
    # rospy.loginfo("Press green A-button for takeoff")
    # while not rospy.is_shutdown():
    #     try:
    #         if joy.buttons[0]==1:
    #             break
    #     except:
    #         pass
    #     rate.sleep()
    cf.takeoff(targetHeight = 0.3, duration = 1.0)
    time.sleep(3.0)
    '''
    cf.goTo(goal = [0.0, -1.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
    time.sleep(3.0)

    cf.goTo(goal = [1.0, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
    time.sleep(3.0)
    
    cf.goTo(goal = [0.0, 1.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
    time.sleep(3.0)

    cf.goTo(goal = [-1.0, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = True)
    time.sleep(3.0)
    '''

    # rospy.loginfo("Press green B-button for land")
    # while not rospy.is_shutdown():
    #     try:
    #         if joy.buttons[1]==1:
    #             break
    #     except:
    #         pass
    #     rate.sleep()
    cf.land(targetHeight = 0.0, duration = 1.0)
    time.sleep(3.0)
    
    cf.stop()


if __name__ == '__main__':
    rospy.init_node('test')
    # joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)
    rate = rospy.Rate(10)

    cf3 = crazyflie.Crazyflie("cf3", "/vicon/cf3/cf3")
    # cf18 = crazyflie.Crazyflie("crazyflie18", "/vicon/crazyflie18/crazyflie18")
    #cf13 = crazyflie.Crazyflie("crazyflie13", "/vicon/crazyflie13/crazyflie13")

    t1 = Thread(target=handler, args=(cf3,))
    # t2 = Thread(target=handler, args=(cf18,))
    #t3 = Thread(target=handler, args=(cf13,))
    
    t1.start()
    # t2.start()
    #t3.start()
