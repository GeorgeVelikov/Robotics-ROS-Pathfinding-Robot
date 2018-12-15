#!/usr/bin/env python

import rospy
import roslib
import tf

from nav_msgs.msg 			import Odometry
from geometry_msgs.msg		import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from marker 				import Markers

class RealRobPos:
	def __init__(self):
		rospy.init_node("realRobotPosition")

		self.p_real 						= Odometry()
		self.p_amcl							= PoseWithCovarianceStamped()
		self.p_broadcaster 					= tf.TransformBroadcaster()
		self.rate 							= rospy.Rate(5)
	
		self.robot 		= Markers(rgb		= [.0, .0, 1.],
					  		  	  ns		= "real_pose",
					  		  	  frame 	= "/map",
					 		 	  size		= [.1, .1, .15],
					 		 	  m_type 	= Marker.CUBE)
		 		 	  
		self.robotAmcl	= Markers(rgb 		= [1., .0, .3],
								  ns		= "amcl_position",
								  frame 	= "/map",
								  size 		= [.1, .1, .1],
								  m_type	= Marker.SPHERE)					  
					 		 
		self.direction	= Markers(rgb		= [.0, 1., .0],
					  		  	  ns		= "real_direction",
					  		  	  frame 	= "/map",
					 		 	  size		= [.125, .03, .03],
					 		 	  m_type 	= Marker.ARROW)			
					 		 	  
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.getPose)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.getAmcl)
			
	def getPose(self, odom):
		self.p_real = odom
		
	def getAmcl(self, amcl):
		self.p_amcl = amcl
		
	def applyPose(self):
		pos 		= self.p_real.pose.pose.position
		ortn		= self.p_real.pose.pose.orientation
		location	= (pos.x, pos.y, pos.z)
		quaternion 	= (ortn.x, ortn.y, ortn.z, ortn.w)
		
		self.p_broadcaster.sendTransform( location, 
										  quaternion, 
										  rospy.Time.now(),
										  '/real_robot_pose',
										  '/map')	
										  
		posAmcl			= self.p_amcl.pose.pose.position
		ortnAmcl		= self.p_amcl.pose.pose.orientation
								 
		self.robot.addMarker(pos, ortn)
		self.direction.addMarker(pos, ortn)
		self.robotAmcl.addMarker(posAmcl,ortnAmcl)
		
	def draw(self):
		self.robot.drawMarkers()
		self.direction.drawMarkers()
		self.robotAmcl.drawMarkers()
	
	def clear(self):
		self.robot.clearMarkers()
		self.direction.clearMarkers()
		self.robotAmcl.clearMarkers()
							
							
#MAIN
rrp = RealRobPos()

try:
	while not rospy.is_shutdown():
		rrp.applyPose()
		rrp.draw()
		rrp.rate.sleep()
		rrp.clear()
		
except rospy.exceptions.ROSInterruptException:
	print "no more error messages on exit"

