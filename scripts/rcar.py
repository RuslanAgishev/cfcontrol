#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.msg import FullState
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path

from math import *
import math
import time
from time import sleep
import matplotlib.pyplot as plt
import numpy as np

import crazyflie
import swarmlib

np.set_printoptions(formatter={'float': '{: 0.2f}'.format})


# PARAMETERs #############
toFly            = 0
pos_ctrl		 = 1
pos_coef         = 3.5
const_height	 = 1
human_imp        = False
theta_imp        = True
delta_imp        = True
force_imp        = False
put_limits       = 0
TAKEOFFHEIGHT    = 1.45 # meters
HUMAN_Z_TO_LAND  = 0.8  # meters
TakeoffTime      = 5    # seconds
l                = 0.40 # distance between drones, meters
R_obstacles      = 0.25
limits           = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_negative  = np.array([-1.7, -1.5, -0.1 ])
rotation 		 = 0
TimeFlightSec    = 10 # [s]

cf_names         = np.array(['cf1',
							 'cf2',
							 'cf3'])
obstacle_names   = np.array([
							 'obstacle0',
							 'obstacle1',
							 'obstacle2',
							 'obstacle3',
							 ])


if __name__ == '__main__':
	rospy.init_node('fly_labirint', anonymous=True)

	if toFly:
		print "takeoff"
		cf1 = crazyflie.Crazyflie(cf_names[0], '/vicon/'+cf_names[0]+'/'+cf_names[0])
		cf1.setParam("commander/enHighLevel", 1)
		cf1.takeoff(targetHeight=TAKEOFFHEIGHT, duration = 2.0)
		time.sleep(TakeoffTime)

	# Objects init
	obstacles = np.array([])
	for i in range(len(obstacle_names)):
		obstacles = np.append(obstacles, swarmlib.Obstacle( obstacle_names[i], i, R_obstacles))
	drone1 = swarmlib.Drone(cf_names[0], obstacles)

	N_samples = 60 * TimeFlightSec
	traj1 = np.array([np.linspace(-2.0, 1.0, N_samples),
					  0.7*np.ones(N_samples),
					  TAKEOFFHEIGHT* np.ones(N_samples)])

	sp_ind = 0
	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		for i in range(len(obstacles)):
			obstacles[i].publish_position()

		drone1.sp = traj1[:,sp_ind]

		if put_limits:
			np.putmask(drone1.sp, drone1.sp >= limits, limits)
			np.putmask(drone1.sp, drone1.sp <= limits_negative, limits_negative)

		# OBSTACLEs
		if theta_imp:
			for i in range(len(obstacles)):
				drone1.sp, updated_1 = swarmlib.pose_update_obstacle_imp(drone1, obstacles[i], R_obstacles, delta_imp=delta_imp)
		else:
			for i in range(len(obstacles)):
				drone1.sp, updated_1 = swarmlib.pose_update_obstacle(drone1, obstacles[i], R_obstacles, delta_imp=delta_imp)

		if sum(drone1.near_obstacle) < 60*4:
			sp_ind += 1

		# TO FLY
		if toFly:
			drone1.fly()

		# TO VISUALIZE
		swarmlib.publish_pose(traj1[:,sp_ind-1], np.array([0,0,0,1]), '/traj1')
		drone1.publish_sp()
		drone1.publish_path(limit=N_samples)

		# Landing
		if sp_ind == N_samples-1:
			print 'Landing!!!'
			drone1.landing(sim=True)

		rate.sleep()

