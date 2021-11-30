#!/usr/bin/env python3

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

def randPos():
	x = np.random.randint(170)/10
	y = -np.random.randint(170)/10
	sample = Odometry()
	sample.pose.pose.position.x = x-2
	sample.pose.pose.position.y = y-2
	return sample.pose.pose

class odom:
	def __init__(self):
		odomSub = rospy.Subscriber('/odom', Odometry, self.call, queue_size = 40)
		self.msg = Odometry()
	def call(self, msg):
		self.msg = msg
	def get_odom(self):
		return self.msg.pose.pose
	
class Node:
	def __init__(self, pose):
		self.pose = pose
		self.neighbors = []
		self.parent = None

class Tree:
	nodes = []
	def __init__(self,start, goal):
		self.start = Node(start)
		self.goal = Node(goal)
		self.nodes.append(self.start)
		
	def getNode(self, pose):
		for i in self.nodes:
			if(i.pose == pose):
				return i
				
	def distance(self, pose1, pose2):
		return np.sqrt((pose1.position.x-pose2.position.x)**2 + (pose1.position.y-pose2.position.y)**2)
		
	def near(self, pose):
		dis = None
		temp = None
		for i in self.nodes:
			if(dis == None or dis > self.distance(i.pose, pose)):
				dis = self.distance(i.pose, pose)
				temp = i
		return i
		
	def add(self, pose1, poseT):
		pose2 = Node(poseT)
		pose1.neighbors.append(pose2)
		pose2.neighbors.append(pose1)
		pose2.parent = pose1
		self.nodes.append(pose2)
		

def inLimit(pos1, pos2):
	if((pos1.position.x > pos2.position.x + .01 or pos1.position.x < pos2.position.x - .01) and (pos1.position.y > pos2.position.y + .01 or pos1.position.y < pos2.position.y - .01)):
		return False
	return True
def inLimitRot(pos1, pos2):
	#print("compare"+str(pos1.orientation.z)+" " +str(pos2.orientation.z))
	if(pos1.orientation.z > pos2.orientation.z + .01 or pos1.orientation.z < pos2.orientation.z - .01):
		return False
	return True
	
def forward(pos, od, vel):
	init = od.get_odom()
	msg = Twist()
	msg.linear.x = 1
	vel.publish(msg)
	rospy.sleep(1)
	last = init
	count = 0
	while(True):
		if(count>30 and inLimit(last, init)):
			return last
		count +=1
		temp = od.get_odom()
		last = temp
		rospy.sleep(.1)
		if(temp.orientation.z > init.orientation.z + .01 or temp.orientation.z < init.orientation.z - .01 or count>300):
			msg = Twist()
			msg.linear.x = 0
			vel.publish(msg)
			rospy.sleep(1)
			stop(last)
			return last
		elif(inLimit(temp, pos)):
			msg = Twist()
			msg.linear.x = 0
			vel.publish(msg)
			rospy.sleep(1)
			stop(pos)
			return pos
		rospy.sleep(.01)
	return last

def rotate(pos, od, vel):
	init = od.get_odom()
	posVector = (pos.position.x - init.position.x, pos.position.y - init.position.y)
	posAngle = (posVector[0])/np.sqrt(posVector[0]**2 + posVector[1]**2)
	ang = np.arccos(posAngle)/3.1415
	pos.orientation.z = ang
	msg = Twist()
	msg.angular.z = .5
	vel.publish(msg)
	rospy.sleep(1)
	last = init
	count = 0
	moved = False
	while(True):
		if(count > 30 and moved ==False):
			return last
		temp = od.get_odom()
		last = temp
		rospy.sleep(.1)
		count += 1
		if(not inLimit(last, init)):
			msg = Twist()
			msg.angular.z = 0
			vel.publish(msg)
			rospy.sleep(1)
			stop(last)
			return last
		elif(inLimitRot(last, pos)):
			msg = Twist()
			msg.angular.z = 0
			vel.publish(msg)
			rospy.sleep(1)
			stop(last)
			return last
		elif(not inLimitRot(last, init)):
			moved = True
		rospy.sleep(.01)
	return last

def stop(pos):
	state_msg = ModelState()
	state_msg.model_name = 'josh'
	state_msg.pose.position.x = pos.position.x
	state_msg.pose.position.y = pos.position.y
	state_msg.pose.position.z = pos.position.z
	state_msg.pose.orientation.x = 0
	state_msg.pose.orientation.y = 0
	state_msg.pose.orientation.z = pos.orientation.z
	state_msg.pose.orientation.w = pos.orientation.w
	rospy.wait_for_service('/gazebo/set_model_state')
	set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	resp = set_state( state_msg )
	rospy.sleep(.1)
	

def resetPose():
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
	


velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
od = odom()
rospy.init_node('pathAlgo')
rospy.sleep(.1)
start = Odometry()
start.pose.pose.position.x = 0
start.pose.pose.position.y = 0
end = Odometry()
end.pose.pose.position.x = 14
end.pose.pose.position.y = -14
tree = Tree(start.pose.pose, end.pose.pose)
rrtIterations = 100
for i in range(rrtIterations):
	sample = randPos()
	nearest = tree.near(sample)
	stop(nearest.pose)
	rot = rotate(nearest.pose,od,velPub)
	temp = forward(rot,od,velPub)
	tree.add(nearest, temp)
	if(tree.distance(tree.goal.pose, temp)<=1):
		print("pathFound")
		break
"""
msg = Odometry()
msg.pose.pose.position.x =0
msg.pose.pose.position.y =1
msg.pose.pose.orientation.z = .5
od.get_odom()
forward(msg.pose.pose, od,velPub)
#rotate(msg.pose.pose, od, velPub)
"""
