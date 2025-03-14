#!/usr/bin/env python
import rospy
import numpy as np
import sys
import sensor_msgs
import struct
import random
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
import tf

# Definition of class
class Estimation_Node:
    
	def __init__(self, node_name):
        
		self.nname = node_name 	#Giving a name for the ROS node
        
		rospy.init_node(self.nname, anonymous=True) #ROS node initialization

		self.num_of_plane_points = 100 # This sets a minimum number of points used to estimate a 3D plane

		self.plane_params = {"red":[-1]*4, "green":[-1]*4, "blue":[-1]*4} # A dictionnary holding the plane parameters, 4 per plane equation ax+by+cz+d = 0

		self.plane_points = {"red":[], "green":[], "blue":[]}

		self.feature_pose = Transform(Vector3(0, 0, 0.5), tf.transformations.quaternion_from_euler(0, 0, 0)) # This will hold the 6DOF pose of the feature, by a 3D vector for the translation and a quaternion for the rotation

		self.linear_solution = [] # This will hold the point of planes intersection obtained by solving a 3x3 linear system of equations

		point_cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.estimate_pose_callback) # ROS topic subscription

		self.br = tf.TransformBroadcaster()

		rospy.spin() # Initiate the ROS loop

	def empty_points(self):
		self.plane_points["red"] = []
		self.plane_points["green"] = []
		self.plane_points["blue"] = []

	def isOrthogonal(self, x, y):
		"""
		Util method checking if 2 planes are approximatly orthogonal
		Approximate orthogonality is defined by an arbitrary threshold
		
		@return boolean
		"""
		return (abs(np.dot(x.T, y)) < 1) # Arbitrary threshold
	
	def checkEnoughPoints(self):
		"""
		Util method which checks if there are enough points to compute the 3 planes

		@return boolean
		"""
		if (self.plane_points["red"] < self.num_of_plane_points):
			return False
		if (self.plane_points["blue"] < self.num_of_plane_points):
			return False
		if (self.plane_points["green"] < self.num_of_plane_points):
			return False
		return True
	
	def isAllOrthogonal(self):
		"""
		Util method checking if the 3 planes computed prior are orthogonal to 
		one another

		@returns boolean
		"""
		isAllOrthogonal = True	
		for (key1, item1) in self.plane_params.items():
			for (key2, item2) in self.plane_params.items():
				if (key1 != key2 and not(self.isOrthogonal(np.array(item1),np.array(item2)))):
					isAllOrthogonal = False
					break
			
		return isAllOrthogonal

	def estimate_pose_callback(self, pointcloud_msg):
		#print 'Received PointCloud2 message. Reading data...'
		point_list = sensor_msgs.point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names = ("x", "y", "z", "rgb"))

		#print 'Retrieving coordinates and colors...'
		for point in point_list:
			rgb = struct.unpack('BBBB', struct.pack('f', point[3]))

			if rgb[2] > 100 and rgb[0] < 20 and rgb[1] < 20: # If dominant red point, concatenate it
				self.plane_points["red"] += [[point[0], point[1], point[2]]]
			elif rgb[1] > 100 and rgb[0] < 20 and rgb[2] < 20: # If dominant green point, concatenate it
				self.plane_points["green"] += [[point[0], point[1], point[2]]]
			elif rgb[0] > 100 and rgb[2] < 20 and rgb[1] < 20: # If dominant blue point, concatenate it
				self.plane_points["blue"] += [[point[0], point[1], point[2]]]

		# Test if there are sufficient points for each plane
		if (not(self.checkEnoughPoints())):
			print("Not enough points to compute intersection")				
			return 
		
		# Estimate the plane equation for each colored point set using Least Squares algorithm
		H = {}
		for (key, item) in self.plane_points.items():
			H[key] = np.array(item)
			y = np.array([-1] * H[key].shape[0]).reshape(-1, 1)
			solution = np.linalg.lstsq(H[key], y)[0]
			self.plane_params[key] = solution


		# Verify that each pair of 3D planes are approximately orthogonal to each other			
		if (not(self.isAllOrthogonal())):
			print("Computed planes are not orthogonal to one another")
			return

		# Feature detection
		# Solve 3x3 linear system of equations given by the three intersecting planes, in order to find their point of intersection
		A = np.transpose(np.hstack((self.plane_params["red"],
				self.plane_params["blue"],
				self.plane_params["green"])))
		
		Y = -1 * np.ones((3,1))
		self.linear_solution = np.linalg.solve(A, Y)

		# Obtain z-axis (blue) vector as the vector orthogonal to the 3D plane defined by the red (x-axis) and the green (y-axis)
		z_axis = self.plane_params["blue"]/np.linalg.norm(self.plane_params["blue"])

		# Obtain y-axis (green) vector as the vector orthogonal to the 3D plane defined by the blue (z-axis) and the red (x-axis)
		y_axis = self.plane_params["green"]/np.linalg.norm(self.plane_params["green"])

		# Obtain x-axis (red) vector as the vector orthogonal to the 3D plane defined by the green (y-axis) and the blue (z-axis)
		x_axis = self.plane_params["red"]/np.linalg.norm(self.plane_params["red"])
		
		# Construct the 3x3 rotation matrix whose columns correspond to the x, y and z axis respectively
		rotation_matrix = np.hstack((x_axis,y_axis,z_axis))

		# Obtain the corresponding euler angles from the previous 3x3 rotation matrix
		euler_angles = tf.transformations.euler_from_matrix(rotation_matrix)

		# Set the translation part of the 6DOF pose 'self.feature_pose'
		self.feature_pose.translation = Vector3(self.linear_solution[0], self.linear_solution[1], self.linear_solution[2])

		# Set the rotation part of the 6DOF pose 'self.feature_pose'
		self.feature_pose.rotation = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])

		# Publish the transform using the data stored in the 'self.feature_pose'
		self.br.sendTransform((self.feature_pose.translation.x, self.feature_pose.translation.y, self.feature_pose.translation.z), self.feature_pose.rotation, rospy.Time.now(), "corner_6dof_pose", "camera_depth_optical_frame") 
		print("Sent message")	

		# Empty points
		self.empty_points()

if __name__ == '__main__':
    my_estim_object = Estimation_Node('my_estimation_node')
