#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
from matplotlib.gridspec import GridSpec
import time
import rospy
import cv2
from cv_bridge import CvBridge

# IMU messages
from sensor_msgs.msg import Imu

# Depth sensor messages
from bar30_depth.msg import Depth

# DVL messages
from rti_dvl.msg import DVL

# Robot state message
from EKF.msg import robot_state

# Images for Sonar data
from sensor_msgs.msg import Image

# Removes axes labels for the specified subplots
def format_axes(fig):
    for i, ax in enumerate(fig.axes):
		if i < 2:
			ax.tick_params(labelbottom=False)

# Class for full single figure display of all relevant BlueROV topics
class BlueROVVisualizer(object):
	def __init__(self):
		self.fig = plt.figure()

		# Setup the grid and assign each plot the proper space
		# Grid was setup as a 6 rows by 2 columns to give proper sizes to the plots
		self.gs = GridSpec(3, 1)
		self.velocity_plot = self.fig.add_subplot(self.gs[0, 0])
		self.depth_plot = self.fig.add_subplot(self.gs[1, 0])
		self.orientation_plot = self.fig.add_subplot(self.gs[2, 0])

		# Add titles and proper spacing to each plot
		self.fig.subplots_adjust(hspace=.5)
		self.velocity_plot.set_title("Velocity")
		self.depth_plot.set_title("Depth")
		self.orientation_plot.set_title("Orientation")
		format_axes(self.fig)

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

		# Variables to store the latest sonar images
		self.M750_img = None
		self.M1200_img = None

		# Variable to store altitude reading from DVL
		self.robot_alt = None

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

	# Plots the newest depth data from the robot state
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

	# Callbakc function to update the M750 with the latest data from the ROS topic
	def M750Callback(self, img):
		# Bridge conversion object to convert between ROS images and opencv images
		bridge = CvBridge()

		# Convert to and opencv message
		# Need to use copy so resize is allowed to manipulate the image
		cv_img = bridge.imgmsg_to_cv2(img).copy()

		# Check to make sure an M1200 image has been received
		# TODO: This throws an error usually on the first first call of the function but it doesn't cause a problem
		if self.M1200_img is not None:
			# Reshape the M750 image to match the shape of the M1200
			cv_img = cv2.resize(cv_img, (self.M1200_img.shape[1], self.M1200_img.shape[0]))

			# Concatenate the two sonar images vertically
			combined_img = cv2.vconcat([cv_img, self.M1200_img])

			# font 
			font = cv2.FONT_HERSHEY_SIMPLEX 
			
			# origin of text
			org = (30, 30) 
			
			# fontScale 
			fontScale = 1
			
			# Green in BGR 
			color = (0, 255, 0) 
			
			# Line thickness of 2 px 
			thickness = 2

			# Draw altitude on tje image to 3 decimal places
			combined_img = cv2.putText(combined_img, "Alt: " + str('%.3f'%self.robot_alt), org, font, fontScale, color, thickness, cv2.LINE_AA)

			# Display the image
			cv2.imshow('Top: M750. Bottom: M1200', combined_img)
			cv2.waitKey(1)

	# Callback function to update the M1200 with the latest data from the ROS topic
	def M1200Callback(self, img):
		# Bridge conversion object to convert between ROS images and opencv images
		bridge = CvBridge()

		# Convert to and opencv message
		cv_img = bridge.imgmsg_to_cv2(img)

		# font 
		font = cv2.FONT_HERSHEY_SIMPLEX 
		
		# origin of text
		org = (30, 30) 
		
		# fontScale 
		fontScale = 1
		
		# Green in BGR 
		color = (0, 255, 0) 
		
		# Line thickness of 2 px 
		thickness = 2

		# Draw depth on the image to 3 decimal places
		cv_img = cv2.putText(cv_img, "Depth: " + str('%.3f'%self.depth_ar[-1]), org, font, fontScale, color, thickness, cv2.LINE_AA)

		# Save Image
		self.M1200_img = cv_img


	# Callback function for the DVL data that updates the current altitude data
	def altitude_callback(self, dvl_msg):
		# Update altitude value
		self.robot_alt = dvl_msg.altitude



if __name__ == "__main__":
	# Set The font sizes for plots 
	plt.rcParams.update({'font.size': 20,
						'legend.fontsize': 10})

	# Initialize the ROS node
	rospy.init_node("kf_visualizer", anonymous=True)

	# Create the visualizer object
	rov_vis = BlueROVVisualizer()

	# Subscriber to robot state
	rospy.Subscriber('imu_filtered_state', robot_state, rov_vis.robotStateCallback)

	# Setup the animated plots with their respective plotting functions
	orientation_ani = animation.FuncAnimation(rov_vis.fig, rov_vis.plotOrientationData, interval=500)
	velocity_ani = animation.FuncAnimation(rov_vis.fig, rov_vis.plotVelData, interval=500)
	depth_ani = animation.FuncAnimation(rov_vis.fig, rov_vis.plotDepthData, interval=500)

	# Subscribers to the sonar images
	rospy.Subscriber('sonar_oculus_node/M750d/image', Image, rov_vis.M750Callback)
	rospy.Subscriber('sonar_oculus_node/M1200d/image', Image, rov_vis.M1200Callback)

	# Subscriber to DVL topic
	rospy.Subscriber('rti/body_velocity/raw', DVL, rov_vis.altitude_callback)

	plt.show()

	rospy.spin()

