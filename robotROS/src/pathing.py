#!/usr/bin/env python3

import rospy
from robotROS.msg import drive
from nav_msgs.msg import Odometry
import numpy as np

class odom:
	def __init__(self):
		odomSub = rospy.Subscriber('/odom', Odometry, self.call, queue_size = 1)
		self.msg = Odometry()
	def call(self, msg):
		self.msg = msg
	def get_odom(self):
		return self.msg.pose.pose

def straight():
	drv = drive()
	drv.forward = 1
	drv.turn = 0
	velPublisher.publish(drv)
	return checkForStop()

def rotate():
	drv = drive()
	drv.forward = 0
	if(np.random.randint(2)==1):
		drv.turn = 1
	velPublisher.publish(drv)
	rospy.sleep(np.random.randint(10,20)/2)
	stop()
	return
	
def back():
	drv = drive()
	drv.forward = -1
	velPublisher.publish(drv)
	rospy.sleep(2)
	stop()
	return

def stop():
	zero = drive()
	zero.forward = 0
	zero.turn = 0
	velPublisher.publish(zero)
	return

def inLimit(pos1, pos2):
	if(pos1.orientation.z < pos2.orientation.z + .03 and pos1.orientation.z > pos2.orientation.z - .03):
		return True
	return False

def checkForStop():
	init = od.get_odom()
	temp = od.get_odom()
	while(inLimit(temp, init)):
		temp = od.get_odom()
		rospy.sleep(.1)
	stop()
	rospy.sleep(.1)
	return temp

rospy.init_node('robotPath')
velPublisher = rospy.Publisher('/drive', drive, queue_size = 2)
stop()

i = 0
while (velPublisher.get_num_connections() == 0):
	if i == 0:
		print("waiting on subscriber")
	i+=1
	i%=5
	rospy.sleep(.5)

for i in range(100):
	rotate()
	straight()
	back()
	rospy.sleep(.1)
