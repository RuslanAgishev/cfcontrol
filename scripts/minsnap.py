#!/usr/bin/env python

import argparse
import rospy
import tf_conversions
from crazyflie_driver.msg import FullState, Position
import geometry_msgs
import uav_trajectory
import pandas as pd
import numpy as np


if __name__ == '__main__':
    rospy.init_node('minsnap')

    rate = rospy.Rate(100)

    msg = FullState()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/world"

    pub = rospy.Publisher("/crazyflie18/cmd_position", Position, queue_size=1)
    start_time = rospy.Time.now()

    TRAJDURATION = 7
    traj = np.array( pd.read_csv('state.csv') )

    for i in range(len(traj)):
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - start_time).to_sec()
        
        msg.x    = traj[i,0]
        msg.y    = traj[i,1]
        msg.z    = traj[i,2]
        msg.yaw = 0

        pub.publish(msg)
        rate.sleep()
