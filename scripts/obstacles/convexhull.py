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
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt


# PARAMETERs #############
toFly            = 0
vel_ctrl         = 0
vel_koef         = 4.0
impedance_on     = False
TAKEOFFHEIGHT    = 1.2 # meters
TakeoffTime      = 5     # seconds
l                = 0.25     # distance between drones, meters
R_obstacles      = 0.27
limits           = np.array([ 2.2, 2.2, 2.5 ]) # np.array([ 2.0, 2.0, 2.5 ])
cf_names         = np.array(['cf1',
							 'cf2',
							 'cf3'])
human_name       = 'palm'
obstacle_names   = np.array([
							 'obstacle1',
							 'obstacle2',
							 'obstacle3',
							 'obstacle4',
							 'obstacle5',
							 'obstacle6',
							 'obstacle7',
							 'obstacle8',
							 'obstacle9',
							 ])
tacile_glove_on  = False

# Variables #########################
initialized = False
prev_pattern_time = time.time()
imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

updated_1 = 0
# SetUp
if tacile_glove_on:
	swarmlib.startXbee()

if __name__ == '__main__':
	rospy.init_node('convexhull', anonymous=True)

	# Objects init
	drone1 = swarmlib.Drone(cf_names[0], leader = True)
	drone2 = swarmlib.Drone(cf_names[1])
	drone3 = swarmlib.Drone(cf_names[2])
	human = swarmlib.Mocap_object(human_name)
	obstacle = np.array([])
	points = np.zeros((len(obstacle_names),2))
	

	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		# TODO: update current position of all objects every loop
		# print '\nhuman_pose', human.position()

		# OBSTACLEs
		# TODO: make it to look good, by Ruslan

		# for i in range(len(obstacle)):
		# 	drone1.sp, updated_1 = swarmlib.pose_update_obstacle(drone1, obstacle[i], R_obstacles)
		# 	drone2.sp, updated_2 = swarmlib.pose_update_obstacle(drone2, obstacle[i], R_obstacles)
		# 	drone3.sp, updated_3 = swarmlib.pose_update_obstacle(drone3, obstacle[i], R_obstacles)

		for i in range(len(obstacle_names)):
			obstacle = np.append(obstacle, swarmlib.Obstacle( obstacle_names[i], np.array([drone1.sp, drone2.sp, drone3.sp]) ) )
			points[i] = obstacle[i].position()[:2]
	
		hull = ConvexHull(points)
		plt.plot(points[:,0], points[:,1], 'o')
		for simplex in hull.simplices:
			plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
		plt.show()

		# TO VISUALIZE
		human.publish_position()
		drone1.publish_sp()
		drone2.publish_sp()
		drone3.publish_sp()
		drone1.publish_path()
		drone2.publish_path()
		drone3.publish_path()
		for i in range(len(obstacle)):
			obstacle[i].publish_position()


		rate.sleep()


