#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray


#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = dict()	# Declaring dictionaries
		self.aruco_marker = dict()

		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):
		i = 0
		for marker in msg.poses:
			self.whycon_marker[i] = [marker.position.x, marker.position.y, marker.position.z]
			i = i+1
		print "WhyCon_marker",self.whycon_marker


	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		
		i = 0
		for marker in msg.markers:
			a = marker.pose.pose
			self.aruco_marker[i] = [a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w]
			i = i+1;
		# Printing the detected markers on terminal
		print "ArUco_marker",self.aruco_marker
		print "\n"




if __name__=="__main__":

	marker = Marker_detect()

	
	while not rospy.is_shutdown():
		rospy.spin()
