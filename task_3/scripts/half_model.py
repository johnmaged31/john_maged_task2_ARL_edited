#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import numpy as np
import time
from std_msgs.msg import String
from std_msgs.msg import Float32

######initial values#####
theta = 2
l = 3
theta = 1
v = 30
rear_wheelX = 0
rear_wheelY = 0
pub = rospy.Publisher('/new_position', String, queue_size=10)

def callback(data):
	global theta,rear_wheelX,rear_wheelY
	#rospy.loginfo("coordiantes are: ")
	rate = rospy.Rate(10)
	delta = float(data.data)
	theta_dot = v*math.tan(delta)/l
	delta_t=1/100
	#rospy.loginfo("%s",delta)
	R = l/math.tan(delta)
	theta = theta + theta_dot*1/100
	y_dot = v*math.sin(theta)
	x_dot = v*math.cos(theta)
	rear_wheelX = rear_wheelX + (x_dot*delta_t)
	rear_wheelY = rear_wheelY + (y_dot*delta_t)	
	coordinates="%s %s %s" % (rear_wheelX,rear_wheelY,theta)
	time.sleep(0.1)
	rospy.loginfo("coordiantes are: %s", coordinates)
	pub.publish(coordinates)
	#rospy.loginfo("ya rab: %s", coordinates)
	rate.sleep()

if __name__ == '__main__':
	pub_initiate = rospy.Publisher('/new_position', String, queue_size=10)
	rospy.init_node('bike_model', anonymous=True)
	init = "0 0 %s" % (theta)
	pub_initiate.publish(init)
	sub = rospy.Subscriber('/to_half_model', Float32, callback)
	rospy.spin()
