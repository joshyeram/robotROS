#!/usr/bin/env python3

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def resetPose():
	rospy.init_node('set_pose')
	state_msg = ModelState()
	state_msg.model_name = 'josh'
	state_msg.pose.position.x = 0
	state_msg.pose.position.y = 0
	state_msg.pose.position.z = .1
	state_msg.pose.orientation.x = 0
	state_msg.pose.orientation.y = 0
	state_msg.pose.orientation.z = 0
	state_msg.pose.orientation.w = 0

	rospy.wait_for_service('/gazebo/set_model_state')

	set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	resp = set_state( state_msg )
	
resetPose()   
