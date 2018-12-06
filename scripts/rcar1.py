#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import *
import math
import time
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from time import sleep
import message_filters
import matplotlib.pyplot as plt
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
from crazyflie_driver.msg import FullState
import geometry_msgs
import tf_conversions
import crazyflie
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})
import swarmlib



# PARAMETERs #############
toFly            = 0
pos_ctrl		 = 1
pos_coef         = 3.5
const_height	 = 1
human_imp        = False
theta_imp        = False
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
TimeFlightSec    = 35 # [s]

cf_names         = np.array(['cf1',
							 'cf2',
							 'cf3'])
obstacle_names   = np.array([
							 'obstacle0',
							 'obstacle1',
							 'obstacle2',
							 'obstacle3',
							 # 'obstacle4',
							 # 'obstacle5',
							 # 'obstacle6',
							 # 'obstacle7',
							 # 'obstacle8',
							 # 'obstacle9',
							 # 'obstacle10',
							 # 'obstacle11',
							 # 'obstacle12',
							 # 'obstacle13',
							 # 'obstacle14',
							 # 'obstacle15',
							 # 'obstacle16',
							 # 'obstacle17',
							 # 'obstacle18',
							 # 'obstacle19',
							 # 'obstacle20',
							 # 'obstacle21',
							 # 'obstacle22',
							 # 'obstacle23',
							 # 'obstacle24',
							 # 'obstacle25'
							 ])


if __name__ == '__main__':
	rospy.init_node('fly_labirint', anonymous=True)

	if toFly:
		print "takeoff"
		cf1 = crazyflie.Crazyflie(cf_names[0], '/vicon/'+cf_names[0]+'/'+cf_names[0])
		cf1.setParam("commander/enHighLevel", 1)
		cf1.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		cf2 = crazyflie.Crazyflie(cf_names[1], '/vicon/'+cf_names[1]+'/'+cf_names[1])
		cf2.setParam("commander/enHighLevel", 1)
		cf2.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		cf3 = crazyflie.Crazyflie(cf_names[2], '/vicon/'+cf_names[2]+'/'+cf_names[2])
		cf3.setParam("commander/enHighLevel", 1)
		cf3.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		time.sleep(TakeoffTime)

	# Objects init
	obstacles = np.array([])
	for i in range(len(obstacle_names)):
		obstacles = np.append(obstacles, swarmlib.Obstacle( obstacle_names[i], i, R_obstacles))
	drones = np.array([])
	for j in range(len(cf_names)):
		drones[j] = np.append( drones, swarmlib.Drone(cf_names[j], obstacles) )

	N_samples = 60 * TimeFlightSec
	traj1 = np.array([np.linspace(-2.0, 1.0, N_samples),
					  0.7*np.ones(N_samples),
					  TAKEOFFHEIGHT* np.ones(N_samples)])
	traj2 = np.array([np.linspace(-2.0, 1.0, N_samples),
					  0.2*np.ones(N_samples),
					  TAKEOFFHEIGHT* np.ones(N_samples)])
	traj3 = np.array([np.linspace(-2.0, 1.0, N_samples),
					  1.2*np.ones(N_samples),
					  TAKEOFFHEIGHT* np.ones(N_samples)])


	sp_ind = 0
	rate = rospy.Rate(60)
	while not rospy.is_shutdown() and sp_ind<N_samples:
		for i in range(len(obstacles)):
			obstacles[i].publish_position()

		drone1.publish_position()
		drone2.publish_position()
		drone3.publish_position()

		drone1.sp = traj1[:,sp_ind]
		drone2.sp = traj2[:,sp_ind]
		drone3.sp = traj3[:,sp_ind]
		sp_ind += 1

		if const_height:
			drone1.sp[2] = TAKEOFFHEIGHT

		if put_limits:
			np.putmask(drone1.sp, drone1.sp >= limits, limits)
			np.putmask(drone1.sp, drone1.sp <= limits_negative, limits_negative)


		# OBSTACLEs
		if theta_imp:
			for i in range(len(obstacles)):
				drone1.sp = swarmlib.pose_update_obstacle_imp(drone1, obstacles[i], R_obstacles, delta_imp=True)
				drone2.sp = swarmlib.pose_update_obstacle_imp(drone2, obstacles[i], R_obstacles, delta_imp=True)
				drone3.sp = swarmlib.pose_update_obstacle_imp(drone3, obstacles[i], R_obstacles, delta_imp=True)
		else:
			for i in range(len(obstacles)):
				drone1.sp, updated_1 = swarmlib.pose_update_obstacle(drone1, obstacles[i], R_obstacles, delta_imp=delta_imp)
				drone2.sp, updated_2 = swarmlib.pose_update_obstacle(drone2, obstacles[i], R_obstacles, delta_imp=delta_imp)
				drone3.sp, updated_3 = swarmlib.pose_update_obstacle(drone3, obstacles[i], R_obstacles, delta_imp=delta_imp)
		# # 2-nd and 3-rd drones avoid the 1-st
		# drone3.sp, updated_3 = swarmlib.pose_update_drone(drone3, drone1, 0.5*l)
		# drone2.sp, updated_2 = swarmlib.pose_update_drone(drone2, drone1, 0.5*l)

		# TO FLY
		if toFly:
			drone1.fly()
			drone2.fly()
			drone3.fly()

		# TO VISUALIZE
		swarmlib.publish_pose(traj1[:,sp_ind-1], np.array([0,0,0,1]), '/traj1')
		print drone1.position()
		swarmlib.publish_arrow(drone1.pose, drone1.orient, '/arrow')
		drone1.publish_sp()
		drone2.publish_sp()
		drone3.publish_sp()
		drone1.publish_path(limit=N_samples)
		drone2.publish_path(limit=N_samples)
		drone3.publish_path(limit=N_samples)

		# Landing
		if toFly and human.position()[2]<HUMAN_Z_TO_LAND:
			print 'Landing!!!'
			drone1_landing_pose = drone1.position()
			drone2_landing_pose = drone2.position()
			drone3_landing_pose = drone3.position()
			while not rospy.is_shutdown():
				drone1.sp = drone1_landing_pose
				drone2.sp = drone2_landing_pose
				drone3.sp = drone3_landing_pose
				drone1_landing_pose[2] = drone1_landing_pose[2]-0.007
				drone2_landing_pose[2] = drone2_landing_pose[2]-0.007
				drone3_landing_pose[2] = drone3_landing_pose[2]-0.007
				drone1.fly()
				drone2.fly()
				drone3.fly()
				if drone1.sp[2]<-1.0 and drone2.sp[2]<-1.0 and drone2.sp[2]<-1.0:
					sleep(1)
					cf1.stop()
					cf2.stop()
					cf3.stop()
					print 'reached the floor, shutdown'
					rospy.signal_shutdown('landed')
				rate.sleep()


		rate.sleep()


