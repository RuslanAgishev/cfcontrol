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
							 # 'obstacle1',
							 # 'obstacle2',
							 # 'obstacle3',
							 # 'obstacle4',
							 # 'obstacle5',
							 # 'obstacle6',
							 # 'obstacle7',
							 # 'obstacle8',
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
	rospy.init_node('follow_multiple', anonymous=True)

	# TODO: drone_list = ['cf1', 'cf2', 'cf3']
	# TODO: swarmlib.SWARM_MANAGER(drone_list)

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
		# time to takeoff and select position for human
		time.sleep(TakeoffTime)

	# Objects init
	drone1 = swarmlib.Drone(cf_names[0], leader = True)
	drone2 = swarmlib.Drone(cf_names[1])
	drone3 = swarmlib.Drone(cf_names[2])
	human = swarmlib.Mocap_object(human_name)
	obstacle = np.array([])
	for i in range(len(obstacle_names)):
		obstacle = np.append(obstacle, swarmlib.Obstacle( obstacle_names[i], np.array([drone1.sp, drone2.sp, drone3.sp]) ) )

	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		# TODO: update current position of all objects every loop
		# print '\nhuman_pose', human.position()

		if impedance_on:
			# HUMAN IMPEDANCE
			hum_vel = swarmlib.hum_vel(human_pose)
			imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
			imp_pose_prev = imp_pose
			imp_vel_prev = imp_vel

		# all drones follow a human with or wo impedance
		if vel_ctrl:
			if not initialized:
				# print "initialized"
				point_to_follow_pose_prev = np.array([drone1.position()[0],drone1.position()[1],TAKEOFFHEIGHT])
				human_pose_init = human.position() # np.array([human_pose[0],human_pose[1],human_pose[2]])
				time_prev = time.time()
				initialized = True
			cmd_vel = -vel_koef*(human_pose_init-human.position())
			np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.020), 0)
			time_now = time.time()
			point_to_follow_pose = point_to_follow_pose_prev + cmd_vel*(time.time()-time_prev)
			time_prev = time_now
			point_to_follow_pose_prev = point_to_follow_pose
			drone1.sp = point_to_follow_pose
		else:
			drone1.sp = np.array([  human.position()[0] -3.0*l        ,
									human.position()[1]               ,
									human.position()[2] + 0.1         ])
		
		np.putmask(drone1.sp, drone1.sp >= limits, limits)
		np.putmask(drone1.sp, drone1.sp <= -limits, -limits)

		drone2.sp = drone1.sp + np.array([-l , l,	0])
		drone3.sp = drone1.sp + np.array([-l ,-l, 0])

		# ROTATION due to hand position
		centroid = swarmlib.centroid_calc(drone1, drone2, drone3)
		drone1.sp = swarmlib.rotate(centroid, drone1, human)
		drone2.sp = swarmlib.rotate(centroid, drone2, human)
		drone3.sp = swarmlib.rotate(centroid, drone3, human)

		# OBSTACLEs
		# TODO: make it to look good, by Ruslan

		# init distances from obstacles to drones
		# drones_poses = np.array([drone1.sp, drone2.sp, drone3.sp])
		# for i in range(len(obstacle)):
		# 	obstacle[i].calculate_dist(drones_poses)

		# for i in range(len(obstacle)):
		# 	drone1.sp, updated_1 = swarmlib.pose_update_obstacle(drone1, obstacle[i], R_obstacles)
		# 	drone2.sp, updated_2 = swarmlib.pose_update_obstacle(drone2, obstacle[i], R_obstacles)
		# 	drone3.sp, updated_3 = swarmlib.pose_update_obstacle(drone3, obstacle[i], R_obstacles)

		drone1.sp = swarmlib.pose_update_obstacle_imp(drone1, obstacle[0], R_obstacles)
		drone2.sp = swarmlib.pose_update_obstacle_imp(drone2, obstacle[0], R_obstacles)
		drone3.sp = swarmlib.pose_update_obstacle_imp(drone3, obstacle[0], R_obstacles)



		# TO FLY
		if toFly:
			drone1.fly()
			drone2.fly()
			drone3.fly()

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

		if tacile_glove_on:
			prev_pattern_time = swarmlib.tactile_patterns(drone1.sp, drone2.sp, drone3.sp, prev_pattern_time)

		rate.sleep()





	print "Try to land"
	# print "land"
	# cf1.land(targetHeight = 0.0, duration = 1.0)
	# cf2.land(targetHeight = 0.0, duration = 1.0)
	# cf3.land(targetHeight = 0.0, duration = 1.0)
	# time.sleep(1.0)
	try:
		cf1.stop()
	except:
		pass
	try:
		cf2.stop()
	except:
		pass
	try:
		cf3.stop()
	except:
		pass





		# drone1.sp = drone1.sp + imp_pose
		# drone2.sp = drone2.sp + imp_pose
		# drone3.sp = drone3.sp + imp_pose
		# drone2.sp[1] = drone2.sp[1] - imp_pose[0]*0.15
		# drone3.sp[1] = drone3.sp[1] + imp_pose[0]*0.15

		# # all drones follow a human
		# drone1.sp = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
		# drone2.sp = np.array([ drone1.sp[0] - l, drone1.sp[1] + l, drone1.sp[2] ])
		# drone3.sp = np.array([ drone1.sp[0] - l, drone1.sp[1] - l, drone1.sp[2] ])

		# # first is followed by the second
		# # Second is followed by the third
		# drone1.sp = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
		# drone2.sp = np.array([ drone1_pose[0] - l, drone1_pose[1] + l, drone1_pose[2] ])
		# drone3.sp = np.array([ drone2_pose[0]    , drone2_pose[1] -2*l, drone2_pose[2] ])