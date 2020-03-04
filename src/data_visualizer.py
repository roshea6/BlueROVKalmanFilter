#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
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

# Class for visualizing robot state data from the IMU
class IMUVisualizer(object):
	# Initialization function to setup variables
	def __init__(self):
		# Variables to be used for the plot
		self.imu_fig, (self.imu_ax1, self.imu_ax2) = plt.subplots(2,1)

		# Set the space between sublots
		self.imu_fig.subplots_adjust(hspace=.4)

		# Set the font size of the axis numbers
		self.imu_ax1.tick_params(labelsize=24)
		self.imu_ax2.tick_params(labelsize=24)

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

		# Max and min y values for the plot
		self.PLOT_Y_MAX = 1.5
		self.PLOT_Y_MIN = -.5

		# Graph interval
		self.INTERVAL = 15


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
	def ekfIMUStateCallback(self, state_msg):
		# print state_msg

		# Get the start time to make graphing look better
		if(self.started == False):
			self.start_time = rospy.Time.now()
			self.started = True

		# Append values from state_msg into their proper arrays
		self.roll_ar.append(state_msg.roll)
		self.pitch_ar.append(state_msg.pitch)
		self.yaw_ar.append(state_msg.yaw)

		# print "Time: " + str(state_msg.header.stamp.secs - self.start_time.secs)

		# Update time array with latest time since start
		self.time_ar.append(state_msg.header.stamp.secs - self.start_time.secs)


	# Recieves messages from the EKF node about the latest state
	def unfiltIMUStateCallback(self, state_msg):
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
	def plotIMUData(self, data):
		# UNFILTERED ROBOT ORIENTATION
		# Clear the previous data
		self.imu_ax1.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.imu_ax1.set_title('Unfiltered Orientation')
		self.imu_ax1.set_ylabel('Euler Angles(rads)', fontsize=30)

		# ! Comment these out to let the graph just do it automatically
		# # Set Y axis range for graph to make it look better
		# self.imu_ax1.set_ylim(bottom=self.PLOT_Y_MIN, top=self.PLOT_Y_MAX) 

		# # Set x axis intervals to make graph look better
		# self.imu_ax1.xaxis.set_major_locator(ticker.MultipleLocator(self.INTERVAL))

		# Plot roll over time
		roll, = self.imu_ax1.plot(self.uf_time_ar, self.uf_roll_ar)
		roll.set_label("Roll") # Label the line

		# Plot pitch over time
		pitch, = self.imu_ax1.plot(self.uf_time_ar, self.uf_pitch_ar)
		pitch.set_label("Pitch") # Label the line

		# Plot yaw over time
		yaw, = self.imu_ax1.plot(self.uf_time_ar, self.uf_yaw_ar)
		yaw.set_label("Yaw") # Label the line

		# Show the legend for the various lines
		self.imu_ax1.legend(loc=2)

		# FILTERED ROBOT ORIENTATION
		# Clear the previous data
		self.imu_ax2.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.imu_ax2.set_title('Filtered Orientation')
		self.imu_ax2.set_xlabel('Time (seconds)')
		self.imu_ax2.set_ylabel('Euler Angles (rads)', fontsize=30)

		# # ! Comment these out to let the graph just do it automatically
		# # Set Y axis range to make graphs look better 
		# self.imu_ax2.set_ylim(bottom=self.PLOT_Y_MIN, top=self.PLOT_Y_MAX)

		# # Set x axis intervals to make graph look better
		# self.imu_ax2.xaxis.set_major_locator(ticker.MultipleLocator(self.INTERVAL))

		# Plot roll over time
		roll, = self.imu_ax2.plot(self.time_ar, self.roll_ar)
		roll.set_label("Roll") # Label the line

		# Plot pitch over time
		pitch, = self.imu_ax2.plot(self.time_ar, self.pitch_ar)
		pitch.set_label("Pitch") # Label the line

		# Plot yaw over time
		yaw, = self.imu_ax2.plot(self.time_ar, self.yaw_ar)
		yaw.set_label("Yaw") # Label the line

		# Show the legend for the various lines
		self.imu_ax2.legend(loc=2)

		# IMU ROBOT ORIENTATION
		# Plot for IMU data
		# self.imu_ax3.clear()

		# self.imu_ax3.set_title('IMU orientation over time')
		# self.imu_ax3.set_xlabel('Time (seconds)')
		# self.imu_ax3.set_ylabel('Euler Angle Values (rads)')

		# # Plot roll over time
		# roll, = self.imu_ax3.plot(self.vn_time_ar, self.vn_roll_ar)
		# roll.set_label("Roll") # Label the line

		# # Plot pitch over time
		# pitch, = self.imu_ax3.plot(self.vn_time_ar, self.vn_pitch_ar)
		# pitch.set_label("Pitch") # Label the line

		# # Plot yaw over time
		# yaw, = self.imu_ax3.plot(self.vn_time_ar, self.vn_yaw_ar)
		# yaw.set_label("Yaw") # Label the line

		# # Show the legend for the various lines
		# self.imu_ax3.legend()

# Class for visualizing robot state data from the DVL
class DVLVisualizer(object):
	# Initialization function to setup variables
	def __init__(self):
		# Variables to be used for the plot
		self.DVL_fig, (self.DVL_ax1, self.DVL_ax2) = plt.subplots(2,1)

		# Set the space between sublots
		self.DVL_fig.subplots_adjust(hspace=.4)

		# Set the font size of the axis numbers
		self.DVL_ax1.tick_params(labelsize=24)
		self.DVL_ax2.tick_params(labelsize=24)

		# Lists for holding values from our state messages
		self.x_dot_ar = []
		self.y_dot_ar = []
		self.z_dot_ar = []
		self.time_ar = []

		# Variables for setting up time to plot against
		self.started = False
		self.start_time = rospy.Time.now()

		# Lists for holding values from our unfiltered state messages
		self.uf_x_dot_ar = []
		self.uf_y_dot_ar = []
		self.uf_z_dot_ar = []
		self.uf_time_ar = []

		# Variables for setting up time to plot against
		self.uf_started = False
		self.uf_start_time = rospy.Time.now()

		# Max and min y values for the plot
		self.PLOT_Y_MAX = .45
		self.PLOT_Y_MIN = -.35

		# Graph interval
		self.INTERVAL = 20

	# Callback function for gathering the filtered DVL data
	def ekfDVLCallback(self, state_msg):
		# Get the start time to make graphing look better
		if(self.started == False):
			self.start_time = rospy.Time.now()
			self.started = True

		# Append values from state_msg into their proper arrays
		self.x_dot_ar.append(state_msg.x_dot)
		self.y_dot_ar.append(state_msg.y_dot)
		self.z_dot_ar.append(state_msg.z_dot)

		# Update time array with latest time since start
		self.time_ar.append(state_msg.header.stamp.secs - self.start_time.secs)


	# Callback function for gathering unfiltered DVL data
	def unfiltDVLCallback(self, state_msg):
		# Get the start time to make graphing look better
		if(self.uf_started == False):
			self.uf_start_time = rospy.Time.now()
			self.uf_started = True

		# Append values from state_msg into their proper arrays
		self.uf_x_dot_ar.append(state_msg.x_dot)
		self.uf_y_dot_ar.append(state_msg.y_dot)
		self.uf_z_dot_ar.append(state_msg.z_dot)

		# Update time array with latest time since start
		self.uf_time_ar.append(state_msg.header.stamp.secs - self.uf_start_time.secs)


	# Plots the newest data from either the DVL or the EKF on the plot
	def plotDVLData(self, data):
		# UNFILTERED ROBOT Velocity
		# Clear the previous data
		self.DVL_ax1.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.DVL_ax1.set_title('Unfiltered Velocity')
		self.DVL_ax1.set_ylabel('Velocity (m/s)')
		
		# ! Comment these out to let the graph just do it automatically
		# Set Y axis range to make graphs look better 
		self.DVL_ax1.set_ylim(bottom=self.PLOT_Y_MIN, top=self.PLOT_Y_MAX)

		# Set x axis intervals to make graph look better
		self.DVL_ax1.xaxis.set_major_locator(ticker.MultipleLocator(self.INTERVAL))


		# Plot x_dot over time
		x_dot, = self.DVL_ax1.plot(self.uf_time_ar, self.uf_x_dot_ar)
		x_dot.set_label("X Vel") # Label the line

		# Plot y_dot over time
		y_dot, = self.DVL_ax1.plot(self.uf_time_ar, self.uf_y_dot_ar)
		y_dot.set_label("Y Vel") # Label the line

		# Plot z_dot over time
		z_dot, = self.DVL_ax1.plot(self.uf_time_ar, self.uf_z_dot_ar)
		z_dot.set_label("Z Vel") # Label the line

		# Show the legend for the various lines
		self.DVL_ax1.legend(loc=2)

		# FILTERED ROBOT Velocity
		# Clear the previous data
		self.DVL_ax2.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.DVL_ax2.set_title('Filtered Velocity')
		self.DVL_ax2.set_xlabel('Time (seconds)')
		self.DVL_ax2.set_ylabel('Velocity (m/s)')

		# ! Comment these out to let the graph just do it automatically
		# Set Y axis range to make graphs look better 
		self.DVL_ax2.set_ylim(bottom=self.PLOT_Y_MIN, top=self.PLOT_Y_MAX)

		# Set x axis intervals to make graph look better
		self.DVL_ax2.xaxis.set_major_locator(ticker.MultipleLocator(self.INTERVAL))


		# Plot x_dot over time
		x_dot, = self.DVL_ax2.plot(self.time_ar, self.x_dot_ar)
		x_dot.set_label("X Vel") # Label the line

		# Plot y_dot over time
		y_dot, = self.DVL_ax2.plot(self.time_ar, self.y_dot_ar)
		y_dot.set_label("Y Vel") # Label the line

		# Plot z_dot over time
		z_dot, = self.DVL_ax2.plot(self.time_ar, self.z_dot_ar)
		z_dot.set_label("Z Vel") # Label the line

		# Show the legend for the various lines
		self.DVL_ax2.legend(loc=2)

# Class for visualizing robot state data from the Depth sensor
class DepthVisualizer(object):
	# Initialization function to setup variables
	def __init__(self):
		# Variables to be used for the plot
		self.Depth_fig, (self.Depth_ax1, self.Depth_ax2) = plt.subplots(2,1)

		# Set the space between sublots
		self.Depth_fig.subplots_adjust(hspace=.4)

		# Set the font size of the axis numbers
		self.Depth_ax1.tick_params(labelsize=24)
		self.Depth_ax2.tick_params(labelsize=24)

		# Lists for holding values from our state messages
		self.depth_ar = []
		self.time_ar = []

		# Variables for setting up time to plot against
		self.started = False
		self.start_time = rospy.Time.now()

		# Lists for holding values from our unfiltered state messages
		self.uf_depth_ar = []
		self.uf_time_ar = []

		# Variables for setting up time to plot against
		self.uf_started = False
		self.uf_start_time = rospy.Time.now()

		# Max and min y values for the plot
		self.PLOT_Y_MAX = 4
		self.PLOT_Y_MIN = -2

		# Graph interval
		self.INTERVAL = 20

	# Callback function for gathering the filtered depth data
	def ekfDepthCallback(self, state_msg):
		# Get the start time to make graphing look better
		if(self.started == False):
			self.start_time = rospy.Time.now()
			self.started = True

		# Append values from state_msg into their proper arrays
		self.depth_ar.append(state_msg.z)

		# Update time array with latest time since start
		self.time_ar.append(state_msg.header.stamp.secs - self.start_time.secs)


	# Callback function for gathering unfiltered Depth data
	def unfiltDepthCallback(self, state_msg):
		# Get the start time to make graphing look better
		if(self.uf_started == False):
			self.uf_start_time = rospy.Time.now()
			self.uf_started = True

		# Append values from state_msg into their proper arrays
		self.uf_depth_ar.append(state_msg.z)

		# Update time array with latest time since start
		self.uf_time_ar.append(state_msg.header.stamp.secs - self.uf_start_time.secs)


	# Plots the newest data from either the Depth or the EKF on the plot
	def plotDepthData(self, data):
		# UNFILTERED ROBOT Velocity
		# Clear the previous data
		self.Depth_ax1.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.Depth_ax1.set_title('Unfiltered Depth')
		self.Depth_ax1.set_ylabel('Depth (m)')
		
		# ! Comment these out to let the graph just do it automatically
		# # Set Y axis range to make graphs look better 
		# self.Depth_ax1.set_ylim(bottom=self.PLOT_Y_MIN, top=self.PLOT_Y_MAX)

		# # Set x axis intervals to make graph look better
		# self.Depth_ax1.xaxis.set_major_locator(ticker.MultipleLocator(self.INTERVAL))


		# Plot depth over time
		depth, = self.Depth_ax1.plot(self.uf_time_ar, self.uf_depth_ar)
		depth.set_label("Depth") # Label the line

		# Show the legend for the various lines
		self.Depth_ax1.legend(loc=2)

		# FILTERED ROBOT Velocity
		# Clear the previous data
		self.Depth_ax2.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.Depth_ax2.set_title('Filtered Depth')
		self.Depth_ax2.set_xlabel('Time (seconds)')
		self.Depth_ax2.set_ylabel('Depth (m)')

		# ! Comment these out to let the graph just do it automatically
		# # Set Y axis range to make graphs look better 
		# self.Depth_ax2.set_ylim(bottom=self.PLOT_Y_MIN, top=self.PLOT_Y_MAX)

		# # Set x axis intervals to make graph look better
		# self.Depth_ax2.xaxis.set_major_locator(ticker.MultipleLocator(self.INTERVAL))


		# Plot x_dot over time
		depth, = self.Depth_ax2.plot(self.time_ar, self.depth_ar)
		depth.set_label("Depth") # Label the line

		# Show the legend for the various lines
		self.Depth_ax2.legend(loc=2)


if __name__ == "__main__":
	# Set The font sizes for plots 
	plt.rcParams.update({'font.size': 34,
						'legend.fontsize': 20})

	# Initialize the ROS node
	rospy.init_node("ekf_visualizer", anonymous=True)

	# Visualizer object which will be used to keep track of the IMU state and plot it
	imu_vis = IMUVisualizer()

	# Subscriber for filtered IMU state 
	rospy.Subscriber('/imu_filtered_state', robot_state, imu_vis.ekfIMUStateCallback)

	# Subsriber for the unfiltered IMU state
	rospy.Subscriber('/imu_unfiltered_state', robot_state, imu_vis.unfiltIMUStateCallback)

	# Subscriber for robot state from pure IMU data
	rospy.Subscriber('/vn_state', robot_state, imu_vis.vnStateCallback)

	# Set the figure to update every second using the plotData function in the Visualizer class
	# The plotData function will clear the previous data and replot with any new data received
	# from the state callback functions
	# * interval must not update the graph faster than the rate at which data is produced or else 
	# * the array of time data will become larger than the array of sensor data and break the graph
	# TODO: Find a workaround for this. Possibly only add time to time array when it won't become larger than sensor data array 
	IMU_ani = animation.FuncAnimation(imu_vis.imu_fig, imu_vis.plotIMUData, interval=1000)

	# Visualizer object which will be used to keep track of the DVL state and plot it
	dvl_vis = DVLVisualizer()

	# Subsriber for filtered DVL state
	rospy.Subscriber('/dvl_filtered_state', robot_state, dvl_vis.ekfDVLCallback)

	# Subscriber for unfiltered DVL state
	rospy.Subscriber('/dvl_unfiltered_state', robot_state, dvl_vis.unfiltDVLCallback)

	# Updating plot for DVL data
	DVL_ani = animation.FuncAnimation(dvl_vis.DVL_fig, dvl_vis.plotDVLData, interval=1000)

	# Visualizer object which will be used to keep track of the DVL state and plot it
	depth_vis = DepthVisualizer()

	# Subsriber for filtered DVL state
	rospy.Subscriber('/depth_filtered_state', robot_state, depth_vis.ekfDepthCallback)

	# Subscriber for unfiltered DVL state
	rospy.Subscriber('/depth_unfiltered_state', robot_state, depth_vis.unfiltDepthCallback)

	# Updating plot for DVL data
	depth_ani = animation.FuncAnimation(depth_vis.Depth_fig, depth_vis.plotDepthData, interval=1000)
	
	
	# Display the plot
	plt.show()

	rospy.spin()

