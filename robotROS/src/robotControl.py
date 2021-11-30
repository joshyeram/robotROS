#!/usr/bin/env python3

import rospy
from robotROS.msg import drive
from geometry_msgs.msg import Twist


def callback(data):
	print(data)
	msg = Twist()
	msg.linear.x = data.forward * .5
	msg.angular.z = data.turn * .3
	pub.publish(msg)
	
	
rospy.init_node('robotControl')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
rospy.Subscriber("/drive", drive, callback)
rospy.spin()
