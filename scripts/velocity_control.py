#!/usr/bin/env python

from __future__ import division
import rospy
import tf
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

toFly = 0

imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

TAKEOFFHEIGHT = 1.0
human_name = 'palm'
cf1_name = 'cf1'
cf2_name = 'cf2'
cf3_name = 'cf3'
initialized = False

# def tag_game(human, cf1):
# def tag_game(human, cf1, cf2):
# def tag_game(human, cf1, cf2, cf3, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7):
# def tag_game(human, cf1, cf2, cf3, cf10_, cf11_):
def tag_game(human, cf1, cf2, cf3, obstacle1):
	print "tag_game"
	global drone1_pose_goal_prev
	global human_pose_init
	global time_prev
	global initialized



	human_pose = swarmlib.get_coord(human)
	drone1_pose = swarmlib.get_coord(cf1)
	drone2_pose = swarmlib.get_coord(cf2)
	drone3_pose = swarmlib.get_coord(cf3)

	obstacle1_pose = swarmlib.get_coord(obstacle1)
	# obstacle2_pose = swarmlib.get_coord(obstacle2)
	# obstacle3_pose = swarmlib.get_coord(obstacle3)
	# obstacle4_pose = swarmlib.get_coord(obstacle4)
	# obstacle5_pose = swarmlib.get_coord(obstacle5)
	# obstacle6_pose = swarmlib.get_coord(obstacle6)
	# obstacle7_pose = swarmlib.get_coord(obstacle7)

	if not initialized:
		print "initialized"
		drone1_pose_goal_prev = np.array([drone1_pose[0],drone1_pose[1],TAKEOFFHEIGHT])
		human_pose_init = np.array([human_pose[0],human_pose[1],human_pose[2]])
		time_prev = time.time()
		initialized = True



	# HUMAN IMPEDANCE
	hum_vel = swarmlib.hum_vel(human_pose)
	global imp_pose_prev
	global imp_vel_prev
	global imp_time_prev
	imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
	imp_pose_prev = imp_pose
	imp_vel_prev = imp_vel

	l = 0.3
	# all drones follow a human with impedance
	# drone1_pose_goal = np.array([  human_pose[0] -3.0*l        ,
	# 							   human_pose[1]               ,
	# 							   human_pose[2] - 0.1         ])

	vel_koef = 2.0
	drone1_vel = -vel_koef*(human_pose_init-human_pose)
	np.putmask(drone1_vel, abs(drone1_vel) <= (vel_koef*0.020), 0)
	time_now = time.time()
	drone1_pose_goal = drone1_pose_goal_prev + drone1_vel*(time.time()-time_prev)
	time_prev = time_now
	drone1_pose_goal_prev = drone1_pose_goal



	drone2_pose_goal = np.array([ drone1_pose_goal[0] - l      ,
								  drone1_pose_goal[1] + l      ,
								  drone1_pose_goal[2]          ])
	drone3_pose_goal = np.array([ drone1_pose_goal[0] - l      ,
								  drone1_pose_goal[1] - l      ,
								  drone1_pose_goal[2]          ])
	# drone1_pose_goal = drone1_pose_goal + imp_pose
	# drone2_pose_goal = drone2_pose_goal + imp_pose
	# drone3_pose_goal = drone3_pose_goal + imp_pose
	# drone2_pose_goal[1] = drone2_pose_goal[1] - imp_pose[0]*0.15
	# drone3_pose_goal[1] = drone3_pose_goal[1] + imp_pose[0]*0.15

	# # all drones follow a human
	# drone1_pose_goal = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
	# drone2_pose_goal = np.array([ drone1_pose_goal[0] - l, drone1_pose_goal[1] + l, drone1_pose_goal[2] ])
	# drone3_pose_goal = np.array([ drone1_pose_goal[0] - l, drone1_pose_goal[1] - l, drone1_pose_goal[2] ])

	# # first is followed by the second
	# # Second is followed by the third
	# drone1_pose_goal = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
	# drone2_pose_goal = np.array([ drone1_pose[0] - l, drone1_pose[1] + l, drone1_pose[2] ])
	# drone3_pose_goal = np.array([ drone2_pose[0]    , drone2_pose[1] -2*l, drone2_pose[2] ])


	# ROTATION due to hand position
	# human_attit = swarmlib.get_angles(human)
	# x_aver = np.array([drone1_pose_goal[0], drone2_pose_goal[0], drone3_pose_goal[0]])
	# y_aver = np.array([drone1_pose_goal[1], drone2_pose_goal[1], drone3_pose_goal[1]])
	# z_aver = np.array([drone1_pose_goal[2], drone2_pose_goal[2], drone3_pose_goal[2]])
	# centroid = np.array([ np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
	# drone1_pose_goal = swarmlib.rotate(centroid, drone1_pose_goal, human_attit[2])
	# drone2_pose_goal = swarmlib.rotate(centroid, drone2_pose_goal, human_attit[2])
	# drone3_pose_goal = swarmlib.rotate(centroid, drone3_pose_goal, human_attit[2])



	# OBSTACLE
	R_obstacles= 0.32
	drone1_pose_goal, updated_1, delta1, theta1_sp = swarmlib.pose_update_obstacle(obstacle1_pose, drone1_pose_goal, R_obstacles)
	drone2_pose_goal, updated_2, delta2, theta2_sp = swarmlib.pose_update_obstacle(obstacle1_pose, drone2_pose_goal, R_obstacles)
	drone3_pose_goal, updated_3, delta3, theta3_sp = swarmlib.pose_update_obstacle(obstacle1_pose, drone3_pose_goal, R_obstacles)

	# # TO FLY
	if toFly:
		swarmlib.publish_goal_pos(drone1_pose_goal, 0, "/"+cf1_name)
		swarmlib.publish_goal_pos(drone2_pose_goal, 0, "/"+cf2_name)
		swarmlib.publish_goal_pos(drone3_pose_goal, 0, "/"+cf3_name)

	# TO VISUALIZE
	swarmlib.publish_pose(human_pose, 0, "human_pose")
	swarmlib.publish_pose(drone1_pose_goal, 0, "drone1_pose_goal")
	swarmlib.publish_pose(drone2_pose_goal, 0, "drone2_pose_goal")
	swarmlib.publish_pose(drone3_pose_goal, 0, "drone3_pose_goal")
	# swarmlib.publish_pose(obstacle1_pose, 0, "obstacle1")
	# swarmlib.publish_pose(obstacle2_pose, 0, "obstacle2")
	# swarmlib.publish_pose(obstacle3_pose, 0, "obstacle3")
	# swarmlib.publish_pose(obstacle4_pose, 0, "obstacle4")
	# swarmlib.publish_pose(obstacle5_pose, 0, "obstacle5")
	# swarmlib.publish_pose(obstacle6_pose, 0, "obstacle6")
	# swarmlib.publish_pose(obstacle7_pose, 0, "obstacle7")







	# ############ TACTILE PATTERNS ##########################

	# AREA calc
	# https://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates
	x = np.array([drone1_pose_goal[0], drone2_pose_goal[0], drone3_pose_goal[0]])
	y = np.array([drone1_pose_goal[1], drone2_pose_goal[1], drone3_pose_goal[1]])
	def PolyArea(x,y):
		return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))
	print 'area', PolyArea(x,y)


	# drone1_pos = drone1_pose_goal
	# drone2_pos = drone2_pose_goal
	# drone3_pos = drone3_pose_goal
	# # print "drone1_pos - drone2_pos", drone1_pos - drone2_pos


	# pos_mid_val = (2 * drone1_pos[0] - drone3_pos[0] - drone2_pos[0]) / 2

	# # boundary = 1.1
	# boundary_max = 1.15
	# boundary_min = 0.85
	# # print "drone1: ", drone1_pos[0], " drone2: ", drone2_pos[0], " drone3: ", drone3_pos[0]

	# global prev_pattern_time
	# # if (time.time()-prev_pattern_time)>0.1:
	# # 	if (pos_mid_val > l*boundary_max):
	# # 		if (not switcher):
	# # 			print "extended, l = ", pos_mid_val
	# # 			swarmlib.send_velocity(6)
	# # 			prev_pattern_time = time.time()
	# # 			switcher = True
	# # 	# elif (pos_mid_val < l*boundary_min):
	# # 	else:
	# # 		if (switcher):
	# # 			print "contracted, l = ", pos_mid_val
	# # 			swarmlib.send_velocity(3)
	# # 			prev_pattern_time = time.time()
	# # 			switcher = False


	# if (time.time()-prev_pattern_time)>0.1:

	# 	if (pos_mid_val > l*boundary_max):
			
	# 		# if (not switcher):
	# 		print "extended, l = ", pos_mid_val
	# 		swarmlib.send_velocity(6)
	# 		prev_pattern_time = time.time()
	# 		# switcher = True

	# 	elif (pos_mid_val < l*boundary_min):
	# 	# else:
			
	# 		# if (switcher):
	# 		print "contracted, l = ", pos_mid_val
	# 		swarmlib.send_velocity(3)
	# 		prev_pattern_time = time.time()
	# 		# switcher = False


	# #############################################################3








def follower():
	
	start_time = time.time()

	human_sub = message_filters.Subscriber('/vicon'+human_name+'/'+human_name, TransformStamped)
	cf1_sub = message_filters.Subscriber('/vicon/'+cf1_name+'/'+cf1_name, TransformStamped)
	cf2_sub = message_filters.Subscriber('/vicon/'+cf2_name+'/'+cf2_name, TransformStamped)
	cf3_sub = message_filters.Subscriber('/vicon/'+cf3_name+'/'+cf3_name, TransformStamped)
	# cf4_sub = message_filters.Subscriber('/vicon/crazyflie4/crazyflie4', TransformStamped)

	obstacle1_sub = message_filters.Subscriber('/vicon/obstacle1/obstacle1', TransformStamped)
	# obstacle2_sub = message_filters.Subscriber('/vicon/obstacle2/obstacle2', TransformStamped)
	# obstacle3_sub = message_filters.Subscriber('/vicon/obstacle3/obstacle3', TransformStamped)
	# obstacle4_sub = message_filters.Subscriber('/vicon/obstacle4/obstacle4', TransformStamped)
	# obstacle5_sub = message_filters.Subscriber('/vicon/obstacle5/obstacle5', TransformStamped)
	# obstacle6_sub = message_filters.Subscriber('/vicon/obstacle6/obstacle6', TransformStamped)
	# obstacle7_sub = message_filters.Subscriber('/vicon/obstacle7/obstacle7', TransformStamped)
	# ts = message_filters.TimeSynchronizer([human_sub, cf1_sub], 10)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, obstacle1_sub, obstacle2_sub, obstacle3_sub, obstacle4_sub, obstacle5_sub, obstacle6_sub, obstacle7_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, cf4_sub], 10, 5)
	ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, obstacle1_sub], 10, 5)

	ts.registerCallback(tag_game)
	rospy.spin()


if __name__ == '__main__':

	rospy.init_node('follow_multiple', anonymous=True)

	if toFly:
		print "takeoff"
		cf1 = crazyflie.Crazyflie(cf1_name, '/vicon/'+cf1_name+'/'+cf1_name)
		cf1.setParam("commander/enHighLevel", 1)
		cf1.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		cf2 = crazyflie.Crazyflie(cf2_name, '/vicon/'+cf2_name+'/'+cf2_name)
		cf2.setParam("commander/enHighLevel", 1)
		cf2.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		cf3 = crazyflie.Crazyflie(cf3_name, '/vicon/'+cf3_name+'/'+cf3_name)
		cf3.setParam("commander/enHighLevel", 1)
		cf3.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
		time.sleep(5.0)



	print "\nfollowing human!\n"
	try:
		follower()
	except KeyboardInterrupt:
		pass





	print "Try to land"

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

