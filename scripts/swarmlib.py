#!/usr/bin/env 

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped


from math import *
import time
from time import sleep
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
import message_filters
import sys
import numpy as np
import serial
from scipy.integrate import odeint

from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position

np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

def msg_def_crazyflie(pose, yaw):
	worldFrame = rospy.get_param("~worldFrame", "/world")
	msg = Position()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.x = pose[0]
	msg.y = pose[1]
	msg.z = pose[2]
	msg.yaw = yaw
	now = rospy.get_time()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()

	return msg

def msg_def_PoseStamped(pose, yaw):
	worldFrame = "world"
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw) #1.57
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.header.seq += 1
	msg.header.stamp = rospy.Time.now()
	return msg

def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
	name = cf_name + "/cmd_position"
	msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
	pub = rospy.Publisher(name, Position, queue_size=1)
	pub.publish(msg)

def get_coord(PoseStamped_message):
	x = PoseStamped_message.transform.translation.x
	y = PoseStamped_message.transform.translation.y
	z = PoseStamped_message.transform.translation.z
	coord_array = np.array([x, y, z])
	return coord_array

def get_angles(PoseStamped_message):
	quat = (
				PoseStamped_message.transform.rotation.x,
				PoseStamped_message.transform.rotation.y,
				PoseStamped_message.transform.rotation.z,
				PoseStamped_message.transform.rotation.w)
	euler = tf.transformations.euler_from_quaternion(quat)
	return euler

def publish_pose(pose, yaw, topic_name):
	msg = msg_def_PoseStamped(pose, yaw)
	pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	pub.publish(msg)







# serial_port = serial.Serial('/dev/ttyUSB0', 9600)
# def send_velocity(pattern):
#     item = '%s\r' % pattern
#     serial_port.write(item.encode())
#     time.sleep(0.025)




# HUMAN VELOCITY CALCULATION
hum_time_array = np.ones(10)
hum_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
def hum_vel(human_pose):

	for i in range(len(hum_time_array)-1):
		hum_time_array[i] = hum_time_array[i+1]
	hum_time_array[-1] = time.time()

	for i in range(len(hum_pose_array[0])-1):
		hum_pose_array[0][i] = hum_pose_array[0][i+1]
		hum_pose_array[1][i] = hum_pose_array[1][i+1]
		hum_pose_array[2][i] = hum_pose_array[2][i+1]
	hum_pose_array[0][-1] = human_pose[0]
	hum_pose_array[1][-1] = human_pose[1]
	hum_pose_array[2][-1] = human_pose[2]

	vel_x = (hum_pose_array[0][-1]-hum_pose_array[0][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_y = (hum_pose_array[1][-1]-hum_pose_array[1][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_z = (hum_pose_array[2][-1]-hum_pose_array[2][0])/(hum_time_array[-1]-hum_time_array[0])

	hum_vel = np.array( [vel_x, vel_y, vel_z] )

	return hum_vel




# HUMAN IMPEDANCE
def MassSpringDamper(state,t,F):
	x = state[0]
	xd = state[1]
	m = 2.0 # Kilograms
	b = 12.6
	k = 20.0 # Newtons per meter
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]

def impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, time_prev):
	F_coeff = 12 # 7
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	F = - hum_vel * F_coeff

	state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
	state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
	state_x = state_x[1]

	state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
	state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
	state_y = state_y[1]

	state0_z = [imp_pose_prev[2], imp_vel_prev[2]]
	state_z = odeint(MassSpringDamper, state0_z, t, args=(F[2],))
	state_z = state_z[1]

	imp_pose = np.array( [state_x[0], state_y[0], state_z[0]] )
	imp_vel  = np.array( [state_x[1], state_y[1], state_z[1]] )

	# return state[0]
	return imp_pose, imp_vel, time_prev



def pose_update_obstacle(obstacle_pose_input, object_pose_input):
	obstacle_pose = np.array([ obstacle_pose_input[0], obstacle_pose_input[1]  ])
	object_pose = np.array([   object_pose_input[0] , object_pose_input[1]  ])
	
	dist = np.linalg.norm(obstacle_pose-object_pose)
	# print "dist", dist

	R = 0.32

	pose_is_updated = False
	
	if dist<R:

		eq1 = np.array([ [obstacle_pose[0],1], [object_pose[0],1] ])
		eq2 = np.array([obstacle_pose[1],object_pose[1]])

		line_equation = np.linalg.solve(eq1,eq2)
		k = line_equation[0]
		b = line_equation[1]

		a_ = k**2+1
		b_ = 2*k*b  - 2*k*obstacle_pose[1] -2*obstacle_pose[0]
		c_ = obstacle_pose[1]**2 - R**2 + obstacle_pose[0]**2 - 2*b*obstacle_pose[1] + b**2

		D = (b_**2) - (4*a_*c_)
		if D>0:
			x_1 = (-b_-sqrt(D))/(2*a_)
			x_2 =  (-b_+sqrt(D))/(2*a_)

		y_1 = k * x_1 + b
		y_2 = k * x_2 + b

		point1 = np.array([ x_1, y_1])
		point2 = np.array([ x_2, y_2])

		dist_point1 = np.linalg.norm(point1 - object_pose)
		dist_point2 = np.linalg.norm(point2 - object_pose)

		if dist_point1 < dist_point2:
			updated_pose = point1
		else:
			updated_pose = point2

		# if point1[1]>point2[1]:
		# 	point_right = point1
		# 	point_left = point2
		# else:
		# 	point_right = point2
		# 	point_left = point1

		# if object_pose[1]>0.5:
		# 	updated_pose = point_right
		# else:
		# 	updated_pose = point_left
		pose_is_updated = True
	else:
		updated_pose = object_pose
		# print "pose not updated"
	

	updated_pose = np.append(updated_pose, object_pose_input[2])

	return updated_pose, pose_is_updated




def pub_circle_traj(x0,y0,z0,r,i):



	# i=0
	# while time_delay<delay:
		
	x1 = x0 + r*sin(i*1.75*pi/360) # 1
	y1 = y0 + r*cos(i*1.75*pi/360) # 1
	z1 = z0

	drone10_pose_goal = np.array([ x1,y1,z1 ])

	x2 = x0 + r*sin(i*1.75*pi/360+pi) # 2
	y2 = y0 + r*cos(i*1.75*pi/360+pi) # 2
	z2 = z0
	
	drone11_pose_goal = np.array([ x2,y2,z2 ])

	i = i+1
	
	# publish_goal_pos(drone10_pose_goal, 0, "/crazyflie10")
	# publish_goal_pos(drone11_pose_goal, 0, "/crazyflie11")

	publish_pose(drone10_pose_goal, 0, "drone10_pose_goal")
	publish_pose(drone11_pose_goal, 0, "drone11_pose_goal")

	return i, drone10_pose_goal, drone11_pose_goal


def lim(val, min, max):
	if val < min:
		val = min
	if val > max:
		val = max
	return val

def limits(val, min, max):
	if val[0] < min:
		val[0] = min
		print "over limit"
	if val[0] > max:
		val[0] = max
		print "over limit"

	if val[1] < min:
		val[1] = min
		print "over limit"
	if val[1] > max:
		val[1] = max
		print "over limit"

	z_limit_down = 0.5

	if val[2] < z_limit_down:
		val[2] = z_limit_down
	if val[2] > 2.5:
		val[2] = 2.5	

	return val

