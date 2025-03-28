#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu


# Definition of class
class ProcessNode:

	def __init__(self, node_name):

		self.nname = node_name 	# Giving a name for the ROS node

		rospy.init_node(self.nname, anonymous=True) #ROS node initialization

		self.imu_data = rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.callback) # ROS topic subscription

		self.motor_power = rospy.Subscriber("/mobile base/commands/motor power", Imu, self.callback) # ROS topic subscription

		rospy.spin() # Initiate the ROS loop
				
		print('Exiting node ' + rospy.get_name()) # This will only be executed if 'rospy.spin()' finishes, after having pressed Ctrl+c

	# Function which is called when a ROS message is heard in the respective ROS topic (see Subscriber initialization further down). The message is stored in the function's argument 'data'
	def callback(self):
		data = self.imu_data
		print(rospy.get_caller_id() + "\n orientation: \n  x: " + str(data.orientation.x) + "\n  y: " + str(data.orientation.y) + "\n  z: " + str(data.orientation.z) + "\n  w: " + str(data.orientation.w) + "\n\n")

if __name__ == '__main__':
    process = ProcessNode('')