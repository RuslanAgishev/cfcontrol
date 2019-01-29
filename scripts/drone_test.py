#!/usr/bin/env python

import numpy as np
import time
import sys

""" ROS """
import rospy
from geometry_msgs.msg import TransformStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
import crazyflie


""" init """
TakeoffHeight = 0.5
TakeoffTime   = 2.0
toFly         = 1

try:
	cf_name = sys.argv[1]
except:
	cf_name = 'cf1'

print(cf_name)

if toFly:
	print("takeoff")
	cf1 = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
	cf1.setParam("commander/enHighLevel", 1)
	cf1.setParam("stabilizer/estimator", 2) # Use EKF
	cf1.setParam("stabilizer/controller", 2) # Use Mellinger controller
	cf1.takeoff(targetHeight = TakeoffHeight, duration = TakeoffTime)
	time.sleep(TakeoffTime+1)

	cf1.land(targetHeight = 0.0, duration = 2.0)
	time.sleep(3.0)
