#!/usr/bin/env python
import rospy
import numpy as np
import tty
import sys
import select
import termios
from geometry_msgs.msg import Twist

# Definition of class
class Teleoperation_Node:
	"""
	Handles control of gazebo turtlebot2 with keyboard control from terminal
	"""

	def __init__(self, node_name):
		"""
		Class constructor : 
		Initializes the node which will publish info on the proper topic
		to control the Turtlebot2. 	
		"""

		# Speed setup
		self.linear_velocity = 1
		self.angular_velocity = 1
		
		# Command speed setup
		self.linear_velocity_cmd = 0
		self.angular_velocity_cmd = 0

		# Setup topic to publish to
		self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
		rospy.init_node(node_name)
		self.rate = rospy.Rate(2)

		# Main control loop
		self.controlBehaviour()
	
		# Final message when discarded
		print("Exiting node : " + node_name)

	def controlBehaviour(self):
		"""
		Main control 
		"""
		while not rospy.is_shutdown():
			key = self.getKey()
			command = self.getCommand(key)
			self.pub.publish(command)
			self.rate.sleep()


	def getCommand(self, key):
		"""
		Creates command to send to Gazebot according to input key
		"""
		
		# Python version too old to use a match case statement

		if (key == "u"):
			self.linear_velocity_cmd = self.linear_velocity
			self.angular_velocity_cmd = 0
		elif (key == "j"):
			self.linear_velocity_cmd = -self.linear_velocity
			self.angular_velocity_cmd = 0
		elif (key == "k"):
			self.linear_velocity_cmd = 0
			self.angular_velocity_cmd = -self.angular_velocity
		elif (key == "h"):
			self.linear_velocity_cmd = 0
			self.angular_velocity_cmd = self.angular_velocity
		elif (key == "f"):
			self.linear_velocity *= 1.10
			self.angular_velocity *= 1.10
		elif (key == "s"):
			self.linear_velocity *= 0.90
			self.angular_velocity *= 0.90

		# Explicit command sent to the topic
		command = Twist()
		command.linear.x = self.linear_velocity_cmd	
		command.angular.z = self.angular_velocity_cmd	
		return command


	def getKey(self):
		"""
		This function reads a single keyboard character from the terminal and returns this character
		"""
		# Back-up default terminal settings
		settings = termios.tcgetattr(sys.stdin)

		tty.setraw(sys.stdin.fileno()) # Setting stdio terminal to raw (no need for pressing enter)
		key = sys.stdin.read(1) # Read 1 character 

		# Restore default terminal settings
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key

if __name__ == '__main__':
    myteleopobject = Teleoperation_Node('my_teleoperation_node')
