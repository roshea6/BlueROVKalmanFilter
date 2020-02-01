# !/usr/bin/env python

import matplotlib.pyplot as plt
import rospy

# IMU messages
from sensor_msgs.msg import Imu

# Depth sensor messages
from bar30_depth.msg import Depth

# DVL messages
from rti_dvl.msg import DVL

# Robot state message
from EKF.msg import robot_state

class visualizer():
	# Initialization function to setup variables
	def __init__(self):
		pass

	# Recieves messages directly from the IMU topic and plots it
	def rawStateCallback(self, imu_msg):
		pass

	# Recieves messages from the 
	def ekfStateCallback(self, imu_msg):
		pass

	# Plots the newest data from either the IMU or the EKF on the plot
	def plotData(self, data):
		pass



if __name__ == "__main__":
	# Initialize the ROS node
	rospy.init_node("ekf_visualizer", anonymous=True)

