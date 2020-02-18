#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import rospy

# IMU messages
from sensor_msgs.msg import Imu

# Depth sensor messages
from bar30_depth.msg import Depth

# DVL messages
from rti_dvl.msg import DVL

# Robot state message
from EKF.msg import robot_state

class Visualizer(object):
	# Initialization function to setup variables
	def __init__(self):
		# Initialize state as a blank robot_state message
		self.state = robot_state()

		# Fill in with dummy values
		self.state.x = 1
		self.state.y = 1
		self.state.z = 1
		self.state.roll = 1
		self.state.pitch = 1
		self.state.yaw = 1
		self.state.x_dot = 1
		self.state.y_dot = 1
		self.state.z_dot = 1
		self.state.roll_dot = 1
		self.state.pitch_dot = 1
		self.state.yaw_dot = 1

		# Variables to be used for the plot
		self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3,1)
		# self.ax1 = self.fig.add_subplot(1,1,1)
		# self.ax2 = self.fig.add_subplot(2,1,1)

		# Lists for holding values from our state messages
		self.roll_ar = []
		self.pitch_ar = []
		self.yaw_ar = []
		self.time_ar = []

		# Variables for setting up time to plot against
		self.started = False
		self.start_time = rospy.Time.now()

		# Lists for holding values from our unfiltered state messages
		self.uf_roll_ar = []
		self.uf_pitch_ar = []
		self.uf_yaw_ar = []
		self.uf_time_ar = []

		# Variables for setting up time to plot against
		self.uf_started = False
		self.uf_start_time = rospy.Time.now()

		# Lists for holding data from vn IMU messages
		self.vn_roll_ar = []
		self.vn_pitch_ar = []
		self.vn_yaw_ar = []
		self.vn_time_ar = []

		# Variables for setting up time to plot against
		self.vn_started = False
		self.vn_start_time = rospy.Time.now()


	# Recieves messages directly from the IMU topic and plots it
	def vnStateCallback(self, imu_msg):
		# Get the start time to make graphing look better
		if(self.vn_started == False):
			self.vn_start_time = rospy.Time.now()
			self.vn_started = True

		# Append values from state_msg into their proper arrays
		self.vn_roll_ar.append(imu_msg.roll)
		self.vn_pitch_ar.append(imu_msg.pitch)
		self.vn_yaw_ar.append(imu_msg.yaw)

		# Update time array with latest time since start
		self.vn_time_ar.append(imu_msg.header.stamp.secs - self.vn_start_time.secs)


	# Recieves messages from the EKF node about the latest state
	def ekfStateCallback(self, state_msg):
		# print state_msg

		# Get the start time to make graphing look better
		if(self.started == False):
			self.start_time = rospy.Time.now()
			self.started = True

		# Update state with new message
		# We might not need this?
		self.state = state_msg

		# Append values from state_msg into their proper arrays
		self.roll_ar.append(state_msg.roll)
		self.pitch_ar.append(state_msg.pitch)
		self.yaw_ar.append(state_msg.yaw)

		# Update time array with latest time since start
		self.time_ar.append(state_msg.header.stamp.secs - self.start_time.secs)


	# Recieves messages from the EKF node about the latest state
	def unfiltStateCallback(self, state_msg):
		# print state_msg

		# Get the start time to make graphing look better
		if(self.uf_started == False):
			self.uf_start_time = rospy.Time.now()
			self.uf_started = True

		# Append values from state_msg into their proper arrays
		self.uf_roll_ar.append(state_msg.roll)
		self.uf_pitch_ar.append(state_msg.pitch)
		self.uf_yaw_ar.append(state_msg.yaw)

		# Update time array with latest time since start
		self.uf_time_ar.append(state_msg.header.stamp.secs - self.uf_start_time.secs)

		



	# Plots the newest data from either the IMU or the EKF on the plot
	def plotData(self, data):

		# FILTERED ROBOT ORIENTATION
		# Clear the previous data
		self.ax1.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.ax1.set_title('Filtered orientation over time')
		# self.ax1.set_xlabel('Time (seconds)')
		self.ax1.set_ylabel('Euler Angle Values (rads)')

		# Plot roll over time
		roll, = self.ax1.plot(self.time_ar, self.roll_ar)
		roll.set_label("Roll") # Label the line

		# Plot pitch over time
		pitch, = self.ax1.plot(self.time_ar, self.pitch_ar)
		pitch.set_label("Pitch") # Label the line

		# Plot yaw over time
		yaw, = self.ax1.plot(self.time_ar, self.yaw_ar)
		yaw.set_label("Yaw") # Label the line

		# Show the legend for the various lines
		self.ax1.legend()

		# UNFILTERED ROBOT ORIENTATION
		# Clear the previous data
		self.ax2.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.ax2.set_title('Unfiltered orientation over time')
		# self.ax1.set_xlabel('Time (seconds)')
		self.ax2.set_ylabel('Euler Angle Values (rads)')

		# Plot roll over time
		roll, = self.ax2.plot(self.uf_time_ar, self.uf_roll_ar)
		roll.set_label("Roll") # Label the line

		# Plot pitch over time
		pitch, = self.ax2.plot(self.uf_time_ar, self.uf_pitch_ar)
		pitch.set_label("Pitch") # Label the line

		# Plot yaw over time
		yaw, = self.ax2.plot(self.uf_time_ar, self.uf_yaw_ar)
		yaw.set_label("Yaw") # Label the line

		# Show the legend for the various lines
		self.ax2.legend()


		# IMU ROBOT ORIENTATION
		# Plot for IMU data
		self.ax3.clear()

		self.ax3.set_title('IMU orientation over time')
		self.ax3.set_xlabel('Time (seconds)')
		self.ax3.set_ylabel('Euler Angle Values (rads)')

		# Plot roll over time
		roll, = self.ax3.plot(self.vn_time_ar, self.vn_roll_ar)
		roll.set_label("Roll") # Label the line

		# Plot pitch over time
		pitch, = self.ax3.plot(self.vn_time_ar, self.vn_pitch_ar)
		pitch.set_label("Pitch") # Label the line

		# Plot yaw over time
		yaw, = self.ax3.plot(self.vn_time_ar, self.vn_yaw_ar)
		yaw.set_label("Yaw") # Label the line

		# Show the legend for the various lines
		self.ax3.legend()





if __name__ == "__main__":
	# Initialize the ROS node
	rospy.init_node("ekf_visualizer", anonymous=True)

	# Visualizer object which will be used to keep track of the robot state and plot it
	vis = Visualizer()

	# Subscriber for our robot state topic
	rospy.Subscriber('/filtered_state', robot_state, vis.ekfStateCallback)

	# Subsriber for the unfiltered state
	rospy.Subscriber('/unfiltered_state', robot_state, vis.unfiltStateCallback)

	# Subscriber for robot state from pure IMU data
	rospy.Subscriber('/vn_state', robot_state, vis.vnStateCallback)

	# TODO: Setup subsriber for raw state

	# Set the figure to update every second using the plotData function in the Visualizer class
	# The plotData function will clear the previous data and replot with any new data received
	# from the state callback functions
	# * interval must not update the graph faster than the rate at which data is produced or else 
	# * the array of time data will become larger than the array of sensor data and break the graph
	# TODO: Find a workaround for this. Possibly only add time to time array when it won't become larger than sensor data array 
	ani = animation.FuncAnimation(vis.fig, vis.plotData, interval=1000)
	
	# Display the plot
	plt.show()

	rospy.spin()

