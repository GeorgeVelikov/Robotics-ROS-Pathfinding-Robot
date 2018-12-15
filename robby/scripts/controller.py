#!/usr/bin/env python

import rospy
import tf

import math

from sensor_msgs.msg		import LaserScan #for obstacle avoidance
from geometry_msgs.msg 		import Twist
from nav_msgs.msg 			import Odometry

class Controller:
	def __init__(self):
		self.speed 			= Twist()
		self.currentPose 	= rospy.get_param("/robot_start")
		self.odomPose		= [0., 0., 0.]
		self.goalMap 		= [0., 0.]
		self.theta 			= 0.
		
		self.publisher = rospy.Publisher("/cmd_vel", 
										 Twist, 
										 queue_size=5)

		rospy.Subscriber("/base_pose_ground_truth", 
						 Odometry, 
						 self.getOdom)
		
	def getDistance(self, rx, ry, gx, gy):
		return math.sqrt((rx - gx)**2 + (ry - gy)**2)

		
	def getOdom(self, odomPose):
		pose 		= odomPose.pose.pose
		position	= pose.position
		orientation	= pose.orientation
		trmnts 		= tf.transformations
		
		posZ = trmnts.euler_from_quaternion([orientation.x,
											 orientation.y,
											 orientation.z, 
											 orientation.w]
											 )[2]
		
		self.currentPose[0] = position.x 
		self.currentPose[1] = position.y 
		self.currentPose[2] = posZ
		self.odomPose 		= [position.x, position.y, posZ]
		
		dx = self.goalMap[0] - self.currentPose[0]
		dy = self.goalMap[1] - self.currentPose[1]
		self.theta			 = math.atan2(dy, dx)
		
	def drive(self, g):	
		rx = self.currentPose[0]
		ry = self.currentPose[1]
		gx = g[0]
		gy = g[1]
		
		self.theta	= math.atan2(gy - ry, gx - rx)
		yaw = self.currentPose[2]
		
		
		dif = self.theta - yaw
		if dif < -math.pi:
			dif += 2*math.pi
		if dif > math.pi:
			dif -= 2*math.pi

		self.speed.angular.z = dif
		
		# create smooth PID
		# fix forward speed to turn speed ratio
		steps = [round(x*.01, 2) for x in range(100)][::-1]
		

		if self.speed.angular.z >= 1:
			self.speed.linear.x = 0
		
		for step in steps:
			if round(self.speed.angular.z, 2) == step:
				self.speed.linear.x = (1. - step)/10
		
		self.publisher.publish(self.speed)

