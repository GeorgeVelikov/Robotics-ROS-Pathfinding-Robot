#!/usr/bin/env python

import rospy
import tf 
import math
import pcl

import controller
import marker
import pathfinding

from nav_msgs.srv 	import GetMap
from nav_msgs.msg 	import Odometry
from Queue			import PriorityQueue


class Robot:
	def __init__(self):
		rospy.init_node("robot")
		self.r_currentPose 	= rospy.get_param("/robot_start")
		self.r_pastPose		= rospy.get_param("/robot_start")
		self.map_cells		= rospy.ServiceProxy('static_map', GetMap)().map
		self.r_controller 	= controller.Controller()
		self.goals 			= self.getGoals()
		self.hz 			= rospy.Rate(10)
		
		rospy.Subscriber("/base_pose_ground_truth", 
						 Odometry, 
						 self.setPose)
		# red is for path
		self.m_path			= marker.Markers(rgb 	= [1., 0., 0.], 
											 ns	 	= "path",
											 frame 	= "/map",
											 size	= [.07, .07, .1])
		# green is for real path						
		self.m_real			= marker.Markers(rgb	= [.0, 1., .0],
											 ns		= "real",
											 frame 	= "/map",
											 size	= [.05, .05, .15])
		# yellow 
		self.m_goals		= marker.Markers(rgb	= [1., 1., 0.],
											 ns 	= "goals",
											 frame 	= "/map",
											 size	= [.07, .07, .07])				 
		# green
		self.m_reached		= marker.Markers(rgb	= [.0, 1., 0.],
											 ns		= "reached",
											 frame 	= "/map",
											 size	= [.1, .1, .1]) 
		goalsQ 				= PriorityQueue()
		
		for goal in self.goals:
				goalsQ.put((0,(goal[0],goal[1])))
				self.m_goals.addMarker(goal)
				
		# Salesman problem for the provided goals
		self.goalsAstar 	= pathfinding.prioritize((self.r_currentPose[0],
										   		  	  self.r_currentPose[1]), 
										   		  	 goalsQ)
		# main
		raw_input("Press any key to start a*")
		
		for aStargoal in self.goalsAstar:
			print "Going to goal:", aStargoal
			
			# change rounds for cellsize
			nextStart	= (round(self.r_currentPose[0], 1), 
						   round(self.r_currentPose[1], 1))
			
			# change rounds for cellsize, actual goal representation in a*
			g = (round(aStargoal[0], 1), round(aStargoal[1], 1))

			pathCurrent = pathfinding.aStar(nextStart, g, self.map_cells)
			
			# adds the precise goal, since path leads to proximity point
			if aStargoal not in pathCurrent:
				pathCurrent.append(aStargoal)
			
			# add lines to all points in the current Path
			self.m_path.addLine(pathCurrent)
			
			# checks every "mini" goal in our path from cp to cp
			for goal in pathCurrent:
				# main 
				while not rospy.is_shutdown():
					rx = self.r_currentPose[0]
					ry = self.r_currentPose[1]
					gx = goal[0]
					gy = goal[1]
					
					# checks if robot has reached "mini" goal
					if abs(rx-gx) + abs(ry-gy) < .12:
						# checks if "mini" goal is cp
						if abs(rx-aStargoal[0]) + abs(ry-aStargoal[1]) < .12:
						  	# notifies when it reaches a main goal in rviz
						  	self.m_reached.addMarker(aStargoal)	
						  	self.m_reached.drawMarkers()				
						break #goto next goal
					
					# rviz
					self.m_real.addMarker([rx,ry])
					self.m_goals.drawMarkers()
					self.m_reached.drawMarkers()
					self.m_path.drawMarkers()
					self.m_real.drawMarkers()
					# pause because vm is trash
					self.hz.sleep()
					# move the robot
					self.r_controller.drive(goal)
					# pauseb ecause vm is trash
					self.hz.sleep()	
					
	# grabs the position from base_pose_ground_truth/real_robot_pose
	def setPose(self, odom):
		pos = odom.pose.pose.position
		self.r_currentPose = (pos.x, pos.y)
	
	# puts all goals from .launch file in a list and adds some prints on xterm
	def getGoals(self):
		points = []
		for i in range(0, 5):
			points.append(rospy.get_param("/goal%d"%i))
			
		print "points:",
		for p in points:
			print p,
		print "\n"
		return points
		
# start instance of robot and run program
robby = Robot()

