#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg 		import Point

class Markers:
	def __init__(self, rgb, ns, frame, size, m_type = Marker.SPHERE):
		self.scale 			= size
		self.id 			= 0
		self.namespace 		= ns
		self.frame 			= frame
		self.type 			= m_type
		self.rgb			= rgb	
		self.markers 		= []	
		self.publisher		= rospy.Publisher(self.namespace,
											  MarkerArray,
											  queue_size = 10)
		Marker.LINE_STRIP
	
	def addMarker(self, pos, ortn = None):
		m = Marker()
		
		m.header.frame_id 	= self.frame
		m.ns				= self.namespace
		m.id				= self.id
		m.type				= self.type
		m.action			= m.ADD
		m.scale.x			= self.scale[0]
		m.scale.y			= self.scale[1]
		m.scale.z			= self.scale[2]		
		m.color.r			= self.rgb[0]
		m.color.g			= self.rgb[1]
		m.color.b			= self.rgb[2]
		m.color.a			= 1.
		
		# could not figure out a better way to combine path and robot markers
		if ortn != None:
			m.pose.position		= pos
			m.pose.orientation	= ortn
			
		else:
			m.pose.position.x	= pos[0]
			m.pose.position.y	= pos[1]

			if len(pos) == 3:
				m.pose.position.z = pos[2]
		
		self.markers.append(m)
		self.id 		   += 1
		
	def addLine(self, points):
		m = Marker()
		
		m.header.frame_id 	= self.frame
		m.ns 				= self.namespace
		m.id				= self.id
		m.type				= m.LINE_STRIP
		m.action			= m.ADD
		m.scale.x			= self.scale[0]
		m.scale.y			= self.scale[1]
		m.scale.z			= self.scale[2]
		m.color.r			= self.rgb[0]
		m.color.g			= self.rgb[1]
		m.color.b			= self.rgb[2]
		m.color.a			= 1.
		
		# iterates over all points in the path to connect them
		for point in points:
			p = Point()
			p.x = point[0]
			p.y = point[1]
			m.points.append(p)
		
		self.markers.append(m)
		self.id			   += 1
		
		
	def drawMarkers(self):
		mArray = MarkerArray()
		
		for marker in self.markers:
			mArray.markers.append(marker)
			
		self.publisher.publish(mArray)
		
	def clearMarkers(self):
		mArray 			= MarkerArray()
		
		if self.markers:
			for marker in self.markers:
				marker.action = marker.DELETE
				mArray.markers.append(marker)
		
		self.publisher.publish(mArray)
		self.id 		= 0
		self.markers 	= []		

