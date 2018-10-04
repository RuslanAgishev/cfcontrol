#!/usr/bin/env python

import rospy
import tf
import crazyflie
import time
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
import numpy as np

def trajgen(crazyflie, mode='setpoint_relative',number_of_points=100, x_des=0,y_des=0,z_des=0):
    x0 = crazyflie.transform.translation.x
    y0 = crazyflie.transform.translation.y
    z0 = crazyflie.transform.translation.z
    if mode=='setpoint_global':
        x = np.linspace(x0,x_des,number_of_points)
        y = np.linspace(y0,y_des,number_of_points)
        z = np.linspace(z0,z_des,number_of_points)
    elif mode=='setpoint_relative':
        x = np.linspace(x0,x0+x_des,number_of_points)
        y = np.linspace(y0,y0+y_des,number_of_points)
        z = np.linspace(z0,z0+z_des,number_of_points)
    return x,y,z


def joy_cb(data):
    global joy
    joy = data

def human_cb(data):
    global human
    human = data

def cf18_cb(data):
    global crazyflie18
    crazyflie18 = data

def cf15_cb(data):
    global crazyflie15
    crazyflie15 = data

if __name__ == '__main__':
    rospy.init_node('follower', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    #cf18 = crazyflie.Crazyflie("crazyflie18", "/vicon/crazyflie18/crazyflie18")
    #cf15 = crazyflie.Crazyflie("crazyflie15", "/vicon/crazyflie15/crazyflie15")
    
    rate = rospy.Rate(20) # 20 hz

    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = 0.0
    msg.y = 0.0
    msg.z = 0.0
    msg.yaw = 0.0

    target_pub18 = rospy.Publisher("/crazyflie18/cmd_position", Position, queue_size=1)
    target_pub15 = rospy.Publisher("/crazyflie15/cmd_position", Position, queue_size=1)
    human_sub = rospy.Subscriber("/vicon/human/human", TransformStamped, human_cb)
    cf18_sub = rospy.Subscriber("/vicon/crazyflie18/crazyflie18", TransformStamped, cf18_cb)
    cf15_sub = rospy.Subscriber("/vicon/crazyflie15/crazyflie15", TransformStamped, cf15_cb)
    joy_sub = rospy.Subscriber("/joy", Joy, joy_cb)

    stop_pub18 = rospy.Publisher("/crazyflie18/cmd_stop", Empty, queue_size=1)
    stop_pub15 = rospy.Publisher("/crazyflie15/cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

    time.sleep(2)
    #HOME-locations:
    x_home18 = crazyflie18.transform.translation.x
    y_home18 = crazyflie18.transform.translation.y
    x_home15 = crazyflie15.transform.translation.x
    y_home15 = crazyflie15.transform.translation.y

    rospy.loginfo('Home locations 18: '+str(x_home18)+', '+str(y_home18))
    rospy.loginfo('Home locations 15: '+str(x_home15)+', '+str(y_home15))

    rospy.loginfo("Press green A-button for takeoff")

    while not rospy.is_shutdown():
        try:
            if joy.buttons[0]==1:
                break
        except:
            pass
        rate.sleep()
    cf5.setParam("commander/enHighLevel", 1)
    cf8.setParam("commander/enHighLevel", 1)
    cf.takeoff(targetHeight = 0.4, duration = 1.0)
    time.sleep(3.0)

    rospy.loginfo("Moving to human location")
    x_human = human.transform.translation.x
    y_human = human.transform.translation.y
    z_human = human.transform.translation.z
    x18,y18,z18 = trajgen(crazyflie18, mode='setpoint_global', x_des=x_human,y_des=y_human-0.5, z_des=z_human+0.3)
    x15,y15,z15 = trajgen(crazyflie15, mode='setpoint_global', x_des=x_human,y_des=y_human-1.0, z_des=z_human+0.3)
    for i in range(len(x18)):
        msg.x = x18[i]
        msg.y = y18[i]
        msg.z = z18[i]
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        target_pub18.publish(msg)
        msg.x = x15[i]
        msg.y = y15[i]
        msg.z = z15[i]
        msg.header.stamp = rospy.Time.now()
        target_pub15.publish(msg)
        rate.sleep()
    DISTANCE = 0.5
    for z in range(20):
        msg.x = x_human
        msg.y = y_human-0.5
        msg.yaw = 0.0
        msg.z = z_human+0.3
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        target_pub18.publish(msg)
        msg.y -= DISTANCE
        msg.header.stamp = rospy.Time.now()
        target_pub15.publish(msg)
        rate.sleep()

    rospy.loginfo("Following human...")
    start = rospy.get_time()
    q = np.zeros(4)
    while not rospy.is_shutdown():
        msg.x = human.transform.translation.x
        msg.y = human.transform.translation.y - 0.5
        #q[0] = human.transform.rotation.x; q[1] = human.transform.rotation.y;
        #q[2] = human.transform.rotation.z; q[3] = human.transform.rotation.w
        #_, _, human_yaw = tf.transformations.euler_from_quaternion(q)
        #msg.yaw = human_yaw * 180 / 3.14
        msg.yaw = 0
        msg.z = human.transform.translation.z + 0.4
        now = rospy.get_time()
        try:
            if joy.buttons[1]==1:
                break
        except:
            pass
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        target_pub18.publish(msg)
        msg.y -= DISTANCE
        msg.header.stamp = rospy.Time.now()
        target_pub15.publish(msg)
        rate.sleep()

    # LOW_LEVEL-land
    rospy.loginfo('Landing...')
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msg.x = human.transform.translation.x
        msg.y = human.transform.translation.y - 0.5
        msg.z -= 0.02
        #msg.yaw = 0.0
        now = rospy.get_time()
        if ( msg.z < -0.05 ):
            break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        target_pub18.publish(msg)
        msg.y -= DISTANCE
        msg.header.stamp = rospy.Time.now()
        target_pub15.publish(msg)
        rate.sleep()

    rospy.loginfo('End')
    stop_pub18.publish(stop_msg)
    stop_pub15.publish(stop_msg)
