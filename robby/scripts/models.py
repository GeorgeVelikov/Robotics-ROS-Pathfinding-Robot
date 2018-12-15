#!/usr/bin/env python

import rospy
import roslib
import tf
import tf2_ros
import cv2
import os
#import pcl #dedicated 2days to get it working, only to find out i dont need it

from tf2_sensor_msgs.tf2_sensor_msgs 	import do_transform_cloud
from nav_msgs.msg 						import Odometry
from sensor_msgs.msg 					import PointCloud2, Image
from sensor_msgs						import point_cloud2
from geometry_msgs.msg					import TransformStamped
from cv_bridge							import CvBridge, CvBridgeError

# python stuff
from ctypes								import c_uint32
from struct								import pack, unpack
from collections 						import OrderedDict

# my own packages
from marker 							import Markers

class Visualiser:
	def __init__(self):
		rospy.init_node("visulisation")
		self.location		= Odometry()

		
		self.m_models		= Markers(rgb		= [.0, .0, .0],
					 			  	  ns		= "models",
					 	 		  	  frame 	= "/map",
					 			 	  size		= [.05, .05, .05])
		
		self.hz 			= rospy.Rate(10)
		self.buffer			= tf2_ros.Buffer()
		self.listener 		= tf2_ros.TransformListener(self.buffer)
		self.bridge			= CvBridge()
		
		# this is where we store the translated points from the pointcloud
		self.pointCloud		= []
		# this is where we store the filtered out points which we plot
		self.realPoints		= []
		
		"""
		# /depth_registered/image_rect
		rospy.Subscriber('/image',  
						 Image, 
						 self.getImage)
		"""
		
		rospy.Subscriber('/rgb_points', 
						 PointCloud2, 
						 self.getPoints)
		
	def getImage(self, img):
		# get current directory
		curdir, curfile = os.path.split(os.path.abspath(__file__))
		try:
			# change image encoding to bgr8, native to cv2
			cv2Image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError, e:
			# only happens whenever your image message is from a bad source
			print "subMsg is not compatible"
		else:
			# need to make sure you have the current directory (30imgs a sec)
			cv2.imwrite(os.path.join(curdir, "image.jpg"), cv2Image)
			# very intensive and could probably add a key trigger
		
	def getPoints(self, pointCloud):	
		pc = pointCloud
		
		# transformation camera -> base_link
		camToBlink			= self.buffer.lookup_transform("base_link", 
														   "camera", 
														   rospy.Time(),
														   rospy.Duration(20.0))
		# applying transformation					   
		baselinkPointCloud	= do_transform_cloud(pc, camToBlink)

		blinkToMap 			= self.buffer.lookup_transform("map", 
														   "real_robot_pose", 
														   rospy.Time(),
														   rospy.Duration(20.0))
		
		mapPointCloud 		= do_transform_cloud(baselinkPointCloud, blinkToMap)		
		
		self.translatePoints(mapPointCloud)
		
	def translatePoints(self, pointCloudTransformed):
		# reads points one by one. This is slow but the only method for py
		pcPoints = point_cloud2.read_points(pointCloudTransformed, 
											skip_nans = True)

		#cointains (x,y,z,r,g,b) value of a point)
		self.realPoints = []
		for point in pcPoints:
			coloursBits = point[3]
			
			# pack the colour intensity bits to a float
			s = pack('>f', coloursBits)
			# unpack the float bits to a long integer representation byte
			i = unpack('>l', s)[0]
			
			# get XYZRGB value of a point
			rgb = c_uint32(i).value
			x = round(point[0],1)
			y = round(point[1],1)
			z = round(point[2],1)
			# bit operations to "translate" the C colour interpretation byte
			r = (rgb & 0x00FF0000) >> 16
			g = (rgb & 0x0000FF00) >> 8
			b = (rgb & 0x000000FF)

			# add the rounded xyz and translated rgb into a list
			xyz = [x,y,z]
			rgb = [r,g,b]
			
			if [xyz,rgb] not in self.realPoints:
				self.realPoints.append([xyz,rgb])	
			# proceed to drawing the scanned points, functions for ease of read
		self.drawPoints()
		
	def drawPoints(self):
	# if there are any scanned points	
		self.hz.sleep()
		self.m_models.clearMarkers()

		for xyz, rgb in self.realPoints:					
			self.m_models.rgb = rgb
			self.m_models.addMarker(xyz)
			
		self.m_models.drawMarkers()
		self.hz.sleep()
			
# creates a map represeantation of what the robo sees
# there is a slight delay from gazebo, this is because of Python's limitations
# and my shit code
v = Visualiser()
rospy.spin()
	
