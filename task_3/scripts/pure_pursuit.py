#!/usr/bin/env python3
# license removed for brevity
import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import time
from std_msgs.msg import String
from std_msgs.msg import Float32

# example for initial conditions
rear_wheelX = 0
rear_wheelY = 0
theta = 2
######initial values######
L = 3
K = 2
#######path equation variables######
y = 0
x = 0
v = 30
i = 0
pub = rospy.Publisher('/to_half_model', Float32, queue_size=10)
def callback(data):
	global rear_wheelX,rear_wheelY,theta
	######### processing data from half model #######
	rear_wheelX,rear_wheelY,theta = data.data.split(' ')
	rear_wheelX = float(rear_wheelX)
	rear_wheelY = float(rear_wheelY)
	theta = float (theta)
	pure_pursuit()
def pure_pursuit():
	global rear_wheelX,rear_wheelY,theta
	#rospy.init_node('user_info_driver', anonymous=True)
	rate = rospy.Rate(10)
	ld = v*K
	x = rear_wheelX
	flag=0
	#rospy.loginfo("saba7 el gamal")
	while(x < 1000 and flag==0):
		y = 100*math.cos(0.1*x)
		distance = math.sqrt(pow((x-rear_wheelX),2)+pow((y-rear_wheelY),2))
		x+=1
		if (distance>(ld-10) and distance<(ld+10)):
			flag=1
	rospy.loginfo("x= %s", x)
	rospy.loginfo("y= %s", y)
	tan_theta_plus_alpha = (y-rear_wheelY)/(x-rear_wheelX)
	if (tan_theta_plus_alpha>0):
		alpha = (math.atan(tan_theta_plus_alpha))-theta
	if (tan_theta_plus_alpha<0):
		alpha = -(math.atan(-tan_theta_plus_alpha))-theta
	sin_alpha = math.sin(alpha)
	delta = math.atan(2*L*sin_alpha)/(ld)
	if (delta>1.5):
		delta = 1.5
	if (delta<-1.5):
		delta = -1.5
	pub.publish(delta)
	rate.sleep()
	rospy.loginfo("delta= %s", delta)
		
if __name__ == '__main__':
	rospy.init_node('data_processing', anonymous=True)
	pub2 = rospy.Publisher('/to_half_model', Float32, queue_size=10)
	pub2.publish(0.1)
	sub = rospy.Subscriber('/new_position', String, callback)
	rospy.spin()
