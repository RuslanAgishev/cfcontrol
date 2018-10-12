#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import *
import time
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from time import sleep
import message_filters
import matplotlib.pyplot as plt

from message_filters import TimeSynchronizer, Subscriber
import numpy as np


import swarmlib

from crazyflie_driver.msg import FullState
import geometry_msgs
import tf_conversions

import crazyflie
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})



imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

imp_delta_pose_prev = np.array( [0,0,0] )
imp_delta_vel_prev = np.array( [0,0,0] )
imp_delta_time_prev = time.time()

imp_theta_prev = 0
imp_omega_prev = 0
imp_theta_time_prev = time.time()

flew_in1 = 0; flew_out1 = 0
flew_in2 = 0; flew_out2 = 0
flew_in3 = 0; flew_out3 = 0




def tag_game(human, cf1, obstacle):
# def tag_game(human, cf1, cf2):
# def tag_game(human, cf1, cf2, cf3, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7):
# def tag_game(human, cf1, cf2, cf3, obstacle):
	human_pose = swarmlib.get_coord(human)
	obstacle_pose = swarmlib.get_coord(obstacle)
	drone1_pose = swarmlib.get_coord(cf1)
	# drone2_pose = swarmlib.get_coord(cf2)
	# drone3_pose = swarmlib.get_coord(cf3)
	# obstacle2_pose = swarmlib.get_coord(obstacle2)
	# obstacle3_pose = swarmlib.get_coord(obstacle3)
	# obstacle4_pose = swarmlib.get_coord(obstacle4)
	# obstacle5_pose = swarmlib.get_coord(obstacle5)
	# obstacle6_pose = swarmlib.get_coord(obstacle6)
	# obstacle7_pose = swarmlib.get_coord(obstacle7)

	l = 0.35
	R_obstacles= 0.32

	hum_vel = swarmlib.hum_vel(human_pose)

	drone1_w, drone1_vel = swarmlib.drone_w(drone1_pose, drone1_pose - obstacle_pose)

	# HUMAN IMPEDANCE
	global imp_pose_prev
	global imp_vel_prev
	global imp_time_prev
	imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
	# print "imp_pose", imp_pose
	imp_pose_prev = imp_pose
	imp_vel_prev = imp_vel
	# swarmlib.publish_pose(human_pose + imp_pose, 0, "human_pose_imp")


	# all drones follow a human with impedance

	# drone1_pose_goal_x = (human_pose[0]-1.703)*1.920/0.900
	# -3*l
	drone1_pose_goal = np.array([  human_pose[0] -3.0*l        ,
								   human_pose[1]               ,
								   human_pose[2] + 0.1         ])
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


	# drone1_pose_goal = swarmlib.pose_update_obstacle(obstacle6_pose, drone1_pose_goal)
	# drone2_pose_goal = swarmlib.pose_update_obstacle(obstacle6_pose, drone2_pose_goal)
	# drone3_pose_goal = swarmlib.pose_update_obstacle(obstacle6_pose, drone3_pose_goal)

	# drone1_pose_goal = swarmlib.pose_update_obstacle(obstacle3_pose, drone1_pose_goal)
	# drone2_pose_goal = swarmlib.pose_update_obstacle(obstacle3_pose, drone2_pose_goal)
	# drone3_pose_goal = swarmlib.pose_update_obstacle(obstacle3_pose, drone3_pose_goal)

	# drone1_pose_goal = swarmlib.pose_update_obstacle(obstacle4_pose, drone1_pose_goal)
	# drone2_pose_goal = swarmlib.pose_update_obstacle(obstacle4_pose, drone2_pose_goal)
	# drone3_pose_goal = swarmlib.pose_update_obstacle(obstacle4_pose, drone3_pose_goal)

	# drone1_pose_goal = swarmlib.pose_update_obstacle(obstacle7_pose, drone1_pose_goal)
	# drone2_pose_goal = swarmlib.pose_update_obstacle(obstacle7_pose, drone2_pose_goal)
	# drone3_pose_goal = swarmlib.pose_update_obstacle(obstacle7_pose, drone3_pose_goal)

	# drone1_pose_goal = swarmlib.pose_update_obstacle(obstacle2_pose, drone1_pose_goal)
	# drone2_pose_goal = swarmlib.pose_update_obstacle(obstacle2_pose, drone2_pose_goal)
	# drone3_pose_goal = swarmlib.pose_update_obstacle(obstacle2_pose, drone3_pose_goal)

	# drone1_pose_goal = swarmlib.pose_update_obstacle(obstacle5_pose, drone1_pose_goal)
	# drone2_pose_goal = swarmlib.pose_update_obstacle(obstacle5_pose, drone2_pose_goal)
	# drone3_pose_goal = swarmlib.pose_update_obstacle(obstacle5_pose, drone3_pose_goal)

	

	drone1_pose_goal, updated_1, delta1, theta1_sp = swarmlib.pose_update_obstacle(obstacle_pose, drone1_pose_goal, R_obstacles)
	drone2_pose_goal, updated_2, delta2, theta2_sp = swarmlib.pose_update_obstacle(obstacle_pose, drone2_pose_goal, R_obstacles)
	drone3_pose_goal, updated_3, delta3, theta3_sp = swarmlib.pose_update_obstacle(obstacle_pose, drone3_pose_goal, R_obstacles)

	# # all drones follow a human
	# drone1_pose_goal = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
	# drone2_pose_goal = np.array([ drone1_pose_goal[0] - l, drone1_pose_goal[1] + l, drone1_pose_goal[2] ])
	# drone3_pose_goal = np.array([ drone1_pose_goal[0] - l, drone1_pose_goal[1] - l, drone1_pose_goal[2] ])

	# # first is followed by the second
	# # Second is followed by the third
	# drone1_pose_goal = np.array([ human_pose[0] - 2*l, human_pose[1]    , human_pose[2] ])
	# drone2_pose_goal = np.array([ drone1_pose[0] - l, drone1_pose[1] + l, drone1_pose[2] ])
	# drone3_pose_goal = np.array([ drone2_pose[0]    , drone2_pose[1] -2*l, drone2_pose[2] ])

	# OBSTACLE DELTA IMPEDANCE
	global imp_delta_pose_prev
	global imp_delta_vel_prev
	global imp_delta_time_prev
	imp_delta_pose, imp_delta_vel, imp_delta_time_prev = swarmlib.impedance_obstacle_delta(delta1, imp_delta_pose_prev, imp_delta_vel_prev, imp_delta_time_prev)
	imp_delta_pose_prev = imp_delta_pose
	imp_delta_vel_prev = imp_delta_vel
	drone1_pose_goal += imp_delta_pose

	# OBSTACLE THETA IMPEDANCE
	# global flew_in1; global flew_out1
	# global flew_in2; global flew_out2
	# global flew_in3; global flew_out3
	# flew_in1, flew_out1 = swarmlib.obstacle_status(obstacle_pose, drone1_pose_goal, human_pose, R_obstacles, flew_in1, flew_out1)
	# global imp_theta_prev
	# global imp_omega_prev
	# global imp_theta_time_prev
	# pub_human_theta = rospy.Publisher('/theta_human', Float32, queue_size=1)
	# pub_drone_theta = rospy.Publisher('/theta_drone', Float32, queue_size=1)
	# # in the circle-like vicinity of the obstacle
	# if flew_in1 > 0:
	# 	if flew_in1 == 1: # crossed the circle near the obstacle
	# 		print "Near the obstacle"
	# 		imp_theta = theta1_sp
	# 		imp_theta_prev = imp_theta
	# 		imp_omega_prev = np.linalg.norm( drone1_w )
	# 	else:
	# 		imp_theta, imp_omega, imp_theta_time_prev = swarmlib.impedance_obstacle_theta(theta1_sp, imp_theta_prev, imp_omega_prev, imp_theta_time_prev)
	# 		imp_theta_prev = imp_theta
	# 		imp_omega_prev = imp_omega
	# 	imp_pose_from_theta = obstacle_pose + np.array([R_obstacles*np.cos(imp_theta), R_obstacles*np.sin(imp_theta), drone1_pose_goal[2]])
	# 	drone1_pose_goal = imp_pose_from_theta
	#print "d_theta="+str( 180/pi*abs( swarmlib.theta_from_pose(drone1_pose_goal, obstacle_pose) - swarmlib.theta_from_pose(human_pose, obstacle_pose) ) )
	# pub_drone_theta.publish( sin(swarmlib.theta_from_pose(drone1_pose_goal, obstacle_pose)) )
	# pub_human_theta.publish( sin(swarmlib.theta_from_pose(human_pose, obstacle_pose)) )

	drone1_w, drone1_vel = swarmlib.drone_w(drone1_pose, drone1_pose_goal-obstacle_pose)
	swarmlib.publish_pose(drone1_w, 0, "drone1_w")

	# TO FLY
	# swarmlib.publish_goal_pos(drone1_pose_goal, 0, "/crazyflie18")
	# swarmlib.publish_goal_pos(drone2_pose_goal, 0, "/crazyflie15")
	# swarmlib.publish_goal_pos(drone3_pose_goal, 0, "/crazyflie13")

	# # TO VISUALIZE
	swarmlib.publish_pose(drone1_pose_goal, 0, "drone1_pose_goal")
	swarmlib.publish_pose(drone2_pose_goal, 0, "drone2_pose_goal")
	swarmlib.publish_pose(drone3_pose_goal, 0, "drone3_pose_goal")
	swarmlib.publish_pose(obstacle_pose, 0, "obstacle")
	swarmlib.publish_pose(human_pose, 0, "human_pose")


def follower():
	start_time = time.time()

	human_sub = message_filters.Subscriber('/vicon/human/human', TransformStamped)
	cf1_sub = message_filters.Subscriber('/vicon/crazyflie18/crazyflie18', TransformStamped)
	cf2_sub = message_filters.Subscriber('/vicon/crazyflie15/crazyflie15', TransformStamped)
	cf3_sub = message_filters.Subscriber('/vicon/crazyflie13/crazyflie13', TransformStamped)

	obstacle_sub = message_filters.Subscriber('/vicon/obstacle/obstacle', TransformStamped)

	# ts = message_filters.TimeSynchronizer([human_sub, cf1_sub], 10)
	ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, obstacle_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, obstacle_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, obstacle1_sub, obstacle2_sub, obstacle3_sub, obstacle4_sub, obstacle5_sub, obstacle6_sub, obstacle7_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, cf4_sub], 10, 5)
	# ts = message_filters.ApproximateTimeSynchronizer([human_sub, cf1_sub, cf2_sub, cf3_sub, cf10_sub, cf11_sub], 10, 5)

	ts.registerCallback(tag_game)
	rospy.spin()


if __name__ == '__main__':
	rospy.init_node('follow_multiple', anonymous=True)

	# TAKEOFFHEIGHT = 0.2
	# print "takeoff at "+str(TAKEOFFHEIGHT)
	# cf1 = crazyflie.Crazyflie("crazyflie18", "/vicon/crazyflie18/crazyflie18")
	# cf1.setParam("commander/enHighLevel", 1)
	# cf1.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
	# cf2 = crazyflie.Crazyflie("crazyflie15", "/vicon/crazyflie15/crazyflie15")
	# cf2.setParam("commander/enHighLevel", 1)
	# cf2.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
	# cf3 = crazyflie.Crazyflie("crazyflie13", "/vicon/crazyflie13/crazyflie13")
	# cf3.setParam("commander/enHighLevel", 1)
	# cf3.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 2.0)
	
	#time.sleep(3.0)



	print "\nfollowing human!\n"
	try:
		follower()
	except KeyboardInterrupt:
		pass

	# print "land"
	# cf1.land(targetHeight = 0.0, duration = 2.0)
	# cf2.land(targetHeight = 0.0, duration = 2.0)
	# cf3.land(targetHeight = 0.0, duration = 2.0)
	# time.sleep(2.0)


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