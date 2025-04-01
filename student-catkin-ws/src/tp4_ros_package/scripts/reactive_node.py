#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from kobuki_msgs.msg import MotorPower   # For controlling motor power
from tf.transformations import euler_from_quaternion

class ReactiveNode:

	def __init__(self, node_name):

		self.nname = node_name 	# Giving a name for the ROS node

		rospy.init_node(self.nname, anonymous=True) #ROS node initialization

		# Keeps history of angles orientation
		self.angleHist = []
		
		# Max amount of data recorded
		self.histSize = 10

		# Threshold in angle variation
		self.angleAccThreshold = 0.05

		self.timeLastCollision = None

		self.motorStopTime = rospy.Time(2)


		rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.callback) # ROS topic subscription

		self.motorPower = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=10)
		
		rospy.spin() # Initiate the ROS loop
				
		print('Exiting node ' + rospy.get_name()) # This will only be executed if 'rospy.spin()' finishes, after having pressed Ctrl+c


	def callback(self, imuData):
		"""
		Main loop

		The purpose of this node is to detect collisions and to 
		cut all motor power when one is detected.

		We then enable back the motor power after enough time 
		has passed .
		"""
		# Saves new imu data
		self.updateHistory(imuData)

		# Need enough data to detect collisions
		if (not(len(self.angleHist) == self.histSize)):
			return
		
		# If no collision is detected now, check time elapsed to activate motors
		if (not(self.checkCollisions())):
			self.handleMotorEnabling()
		
		# Collision has been detected
		self.handleCollisionBehaviour()
		

	def handleMotorEnabling(self):
		"""
		In case a collision has been detected before,
		enables the motors after a certain amount of time
		"""
		# No collision so no need to activate the motors again
		if (self.timeLastCollision is None):
			return

		# User must wait a certain amount of time before motors are active once again
		if (rospy.Time.now() >= self.timeLastCollision + self.motorStopTime):
			self.timeLastCollision = None
			self.startMotors()
			print(rospy.get_caller_id() + " > " + self.motorStopTime.to_sec() + " seconds have elapsed. Enabling motors")

	def handleCollisionBehaviour (self):
		"""
		Updates time since last collision and
		stop motors
		"""
		print(rospy.get_caller_id() + " > Collision detected !")
		if (self.timeLastCollision is None):
			self.timeLastCollision = rospy.Time.now()

		print(rospy.get_caller_id() + " > Motors shutting down for : " + self.motorStopTime.to_sec() + " seconds")
		self.stopMotors()

	def updateHistory(self, imuData):
		"""
		Converts latest imu data to euler angles and
		adds it to history
		"""
		rx, ry, _ = euler_from_quaternion([imuData.orientation.x,
									  		imuData.orientation.y,
											imuData.orientation.z,
											imuData.orientation.w])

		if (len(self.angleHist) >= self.histSize):
			self.angleHist.pop(0)
		
		self.angleHist.append(np.array([rx, ry]))

	def checkCollisions(self):
		"""
		Checks for collisions with the robot

		We take the mean of the history recorded and if the last data 
		is too far to the mean, we detect a collision

		"""
		lastAngle = self.angleHist[-1]
		angleMean = np.mean(np.array(self.angleHist[:-1]), axis=0)

		if (np.linalg.norm(angleMean - lastAngle) > self.angleAccThreshold):
			return True
		return False


	def stopMotors(self):
		motorCommand = MotorPower()
		motorCommand.state = MotorPower.OFF

		self.motorPower.publish(motorCommand)

	def startMotors(self):
		motorCommand = MotorPower()
		motorCommand.state = MotorPower.ON

		self.motorPower.publish(motorCommand)


if __name__ == '__main__':
    process = ReactiveNode('process_node')