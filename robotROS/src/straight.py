#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def straight(dis):
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
	rospy.init_node('straight')
	rospy.sleep(.1)
	msg = Twist()
	msg.linear.x = dis
	pub.publish(msg)
	rospy.sleep(1)
	
	
straight(1)
