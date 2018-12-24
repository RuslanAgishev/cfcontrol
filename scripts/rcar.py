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
from scipy.spatial.distance import cdist

import crazyflie
import swarmlib

np.set_printoptions(formatter={'float': '{: 0.2f}'.format})


def intersection(curve1, curve2):
	dists = cdist(curve1,curve2)
	i1, i2 = np.where(dists==np.min(dists))
	i1 = i1[0]; i2 = i2[0]
	return np.array([curve1[i1,0], curve1[i1,1]])


# PARAMETERs #############
toFly            = 0
pos_ctrl		 = 1
pos_coef         = 3.5
const_height	 = 1
human_imp        = False
ang_imp          = True
rad_imp          = True
force_imp        = False
put_limits       = 0
TAKEOFFHEIGHT    = 0.8 # meters
HUMAN_Z_TO_LAND  = 0.8  # meters
TakeoffTime      = 5    # seconds
R_obstacles      = 0.25
R_swarm          = 0.20
limits           = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_negative  = np.array([-1.7, -1.5, -0.1 ])
rotation 		 = 0
TimeFlightSec    = 10 # [s]
ViconRate        = 100 # [Hz]

cf_names         = np.array(['cf1',
							 'cf2',
							 'cf3'
							 ])
obstacle_names   = np.array([
							 'obstacle0',
							 # 'obstacle1',
							 # 'obstacle2',
							 'obstacle3',
							 'obstacle4',
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
	drone2 = swarmlib.Drone(cf_names[1], obstacles)
	drone3 = swarmlib.Drone(cf_names[2], obstacles)
	centroid = swarmlib.Centroid(obstacles)

	centroid.start = obstacles[0].position() + np.array([-1.5,0,TAKEOFFHEIGHT])
	centroid.goal  = obstacles[2].position() + np.array([1,0,TAKEOFFHEIGHT])
	xs, ys, zs = centroid.start
	xg, yg, zg = centroid.goal
	N_samples = ViconRate * TimeFlightSec
				    #                  [X]                                    [Y]                            [Z]
	centroid.traj = np.array([np.linspace(xs, xg, N_samples), np.linspace(ys, yg, N_samples), np.linspace(zs, zg, N_samples)]).T

	sp_ind = 0
	rate = rospy.Rate(ViconRate)
	while not rospy.is_shutdown():
		for i in range(len(obstacles)):
			obstacles[i].publish_position()

		centroid.sp = centroid.traj[sp_ind,:]

		if put_limits:
			np.putmask(drone1.sp, drone1.sp >= limits, limits)
			np.putmask(drone1.sp, drone1.sp <= limits_negative, limits_negative)

		near = sum( abs(centroid.d_theta)>pi/16. )>0
		if near:
			centroid.sp = centroid.feature
		else:
			sp_ind += 1

		# OBSTACLEs
		if ang_imp:
			for i in range(len(obstacles)):
				centroid.sp, updated_1 = swarmlib.pose_update_obstacle_imp(centroid, obstacles[i], R_obstacles+R_swarm, rad_imp=rad_imp, rad_imp_koef=0.15)
		else:
			for i in range(len(obstacles)):
				centroid.sp, updated_1 = swarmlib.pose_update_obstacle(centroid, obstacles[i], R_obstacles+R_swarm, rad_imp=rad_imp, rad_imp_koef=0.3)

		""" drones forming equilateral triangle """
		drone1.sp = centroid.sp + np.array([R_swarm,     0,                   0])
		drone2.sp = centroid.sp + np.array([-R_swarm/2., R_swarm*sqrt(3)/2.,  0])
		drone3.sp = centroid.sp + np.array([-R_swarm/2., -R_swarm*sqrt(3)/2., 0])

		# TO FLY
		if toFly:
			drone1.fly()
			drone2.fly()
			drone3.fly()

		# TO VISUALIZE
		""" setpoints """
		swarmlib.publish_pose(centroid.feature, np.array([0,0,0,1]), '/feature_centroid')
		centroid.publish_sp()
		drone1.publish_sp()
		drone2.publish_sp()
		drone3.publish_sp()

		""" paths """
		centroid.publish_path(limit=N_samples)
		drone1.publish_path(limit=N_samples)
		drone2.publish_path(limit=N_samples)
		drone3.publish_path(limit=N_samples)

		# if sp_ind >= N_samples-1: sp_ind = N_samples-1 # holding the last pose
		""" Landing """
		if sp_ind == N_samples-1:
			print 'Landing!!!'
			drone1_landing_pose = drone1.sp
			drone2_landing_pose = drone2.sp
			drone3_landing_pose = drone3.sp
			while not rospy.is_shutdown():
				drone1.sp = drone1_landing_pose
				drone2.sp = drone2_landing_pose
				drone3.sp = drone3_landing_pose
				drone1_landing_pose[2] = drone1_landing_pose[2]-0.007
				drone2_landing_pose[2] = drone2_landing_pose[2]-0.007
				drone3_landing_pose[2] = drone3_landing_pose[2]-0.007
				if toFly:
					drone1.fly()
					drone2.fly()
					drone3.fly()
				drone1.publish_sp()
				drone2.publish_sp()
				drone3.publish_sp()
				if drone1.sp[2]<-1.0 and drone2.sp[2]<-1.0 and drone2.sp[2]<-1.0:
					sleep(1)
					if toFly:
						cf1.stop()
						cf2.stop()
						cf3.stop()
					print 'reached the floor, shutdown'
					rospy.signal_shutdown('landed')
				rate.sleep()

		rate.sleep()
