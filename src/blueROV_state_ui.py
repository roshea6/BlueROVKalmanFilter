#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
from matplotlib.gridspec import GridSpec
import time
import rospy
import cv2
import numpy as np

# IMU messages
from sensor_msgs.msg import Imu

# Depth sensor messages
from bar30_depth.msg import Depth

# DVL messages
from rti_dvl.msg import DVL

# Robot state message
from EKF.msg import robot_state

# Removes axes labels for the specified subplots
def format_axes(fig):
    for i, ax in enumerate(fig.axes):
		if i != 4 and i < 2:
			ax.tick_params(labelbottom=False, labelleft=False)
		elif i != 4:
			ax.tick_params(labelbottom=False)

# Class for full single figure display of all relevant BlueROV topics
class BlueROVVisualizer(object):
	def __init__(self):
		self.fig = plt.figure()

		# Setup the grid and assign each plot the proper space
		# Grid was setup as a 6 rows by 2 columns to give proper sizes to the plots
		self.gs = GridSpec(6, 2)
		self.M750_plot = self.fig.add_subplot(self.gs[0:3, 0])
		self.M1200_plot = self.fig.add_subplot(self.gs[3:6, 0])
		self.velocity_plot = self.fig.add_subplot(self.gs[0:2, 1])
		self.depth_plot = self.fig.add_subplot(self.gs[2:4, 1])
		self.orientation_plot = self.fig.add_subplot(self.gs[4:6, 1])

		# Add titles and proper spacing to each plot
		self.fig.subplots_adjust(hspace=.9)
		self.M750_plot.set_title("M750")
		self.M1200_plot.set_title("M1200")
		self.velocity_plot.set_title("Velocity")
		self.depth_plot.set_title("Depth")
		self.orientation_plot.set_title("Orientation")
		format_axes(self.fig)

		# self.img = cv2.imread('../images/rviz_bag_vis.png')
		# self.M750_plot.imshow(self.img)

		# Variables for holding state data to plot
		# Orientation data
		self.roll_ar = []
		self.pitch_ar = []
		self.yaw_ar = []

		# Velocity data
		self.x_dot_ar = []
		self.y_dot_ar = []
		self.z_dot_ar = []

		# Depth data
		self.depth_ar = []

		# Time array to hold timestamp data to plot against
		self.time_ar = []

		# Variables for setting up time to plot against
		self.start_time = rospy.Time.now()

	# Recieves messages from the EKF node about the latest state
	def robotStateCallback(self, state_msg):
		# Append orientation values into their proper arrays
		self.roll_ar.append(state_msg.roll)
		self.pitch_ar.append(state_msg.pitch)
		self.yaw_ar.append(state_msg.yaw)

		# Append velocity values into their proper arrays
		self.x_dot_ar.append(state_msg.x_dot)
		self.y_dot_ar.append(state_msg.y_dot)
		self.z_dot_ar.append(state_msg.z_dot)

		# Append depth data into the proper array
		self.depth_ar.append(state_msg.z)

		# Update time array with latest time since start
		self.time_ar.append(state_msg.header.stamp.secs - self.start_time.secs)

	# Plots the newest oreintation data 
	def plotOrientationData(self, data):
		# Clear the previous data
		self.orientation_plot.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.orientation_plot.set_title('Robot Orientation')
		self.orientation_plot.set_xlabel('Time (seconds)')
		self.orientation_plot.set_ylabel('Euler Angles (rads)', fontsize=20)

		# Plot roll over time
		roll, = self.orientation_plot.plot(self.time_ar, self.roll_ar)
		roll.set_label("Roll") # Label the line

		# Plot pitch over time
		pitch, = self.orientation_plot.plot(self.time_ar, self.pitch_ar)
		pitch.set_label("Pitch") # Label the line

		# Plot yaw over time
		yaw, = self.orientation_plot.plot(self.time_ar, self.yaw_ar)
		yaw.set_label("Yaw") # Label the line

		# Show the legend for the various lines
		self.orientation_plot.legend()

	# Plots the newest velocity data
	def plotVelData(self, data):
		# Clear the previous data
		self.velocity_plot.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.velocity_plot.set_title('Robot Velocity')
		self.velocity_plot.set_ylabel('Velocity (m/s)')

		# Plot x_dot over time
		x_dot, = self.velocity_plot.plot(self.time_ar, self.x_dot_ar)
		x_dot.set_label("X Vel") # Label the line

		# Plot y_dot over time
		y_dot, = self.velocity_plot.plot(self.time_ar, self.y_dot_ar)
		y_dot.set_label("Y Vel") # Label the line

		# Plot z_dot over time
		z_dot, = self.velocity_plot.plot(self.time_ar, self.z_dot_ar)
		z_dot.set_label("Z Vel") # Label the line

		# Show the legend for the various lines
		self.velocity_plot.legend()

	# Plots the newest data from either the Depth or the EKF on the plot
	def plotDepthData(self, data):
		# FILTERED ROBOT Depth
		# Clear the previous data
		self.depth_plot.clear()

		# Set title and lables for the plot
		# TODO: Find a way to only have the plotted data cleared so we don't have to redo these every time
		self.depth_plot.set_title('Depth')
		self.depth_plot.set_ylabel('Depth (m)')

		# Plot x_dot over time
		depth, = self.depth_plot.plot(self.time_ar, self.depth_ar)
		depth.set_label("Depth") # Label the line

		# Show the legend for the various lines
		self.depth_plot.legend()


if __name__ == "__main__":
	# Set The font sizes for plots 
	plt.rcParams.update({'font.size': 20,
						'legend.fontsize': 10})

	# Initialize the ROS node
	rospy.init_node("ekf_visualizer", anonymous=True)

	rov_vis = BlueROVVisualizer()

	# Subscriber to robot state
	rospy.Subscriber('imu_filtered_state', robot_state, rov_vis.robotStateCallback)

	# Setup the animated plots with their respective plotting functions
	orientation_ani = animation.FuncAnimation(rov_vis.fig, rov_vis.plotOrientationData, interval=1000)
	velocity_ani = animation.FuncAnimation(rov_vis.fig, rov_vis.plotVelData, interval=1000)
	depth_ani = animation.FuncAnimation(rov_vis.fig, rov_vis.plotDepthData, interval=1000)

	plt.show()

	rospy.spin()

