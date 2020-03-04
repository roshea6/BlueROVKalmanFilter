#!/usr/bin/env python

import rospy
import numpy as np

# IMU messages
from sensor_msgs.msg import Imu

# Depth sensor messages
from bar30_depth.msg import Depth

# DVL messages
from rti_dvl.msg import DVL

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Robot state message
from EKF.msg import robot_state

#  Turns robot state messages into messages that RVIZ can more easily display
class rviz_visualizer(object):
	# Initialization of variables and objects
	def __init__(self):
		# Publisher for odometry data to be visualized in RVIZ
		self.odom_pub = rospy.Publisher("/imu_odom", Odometry, queue_size=100)

		# Publisher for path data
		self.path_pub = rospy.Publisher("/robot_path", Path, queue_size=1)

		# Variable to store path data in. Will be updated every callback call
		self.path = Path()

	# Callback function to turn state messages into odometry messages
	def imu_state_callback(self, state_msg):
		# Create a new odometry message
		odom = Odometry()

		# Fill the header with the proper timestamp and frame id
		odom.header.stamp = state_msg.header.stamp
		odom.header.frame_id = "map"

		# Get x, y, and z from state msg
		odom.pose.pose.position.x = state_msg.x
		odom.pose.pose.position.y = state_msg.y
		odom.pose.pose.position.z = -state_msg.z # Negative because positive depth = distance below the surface

		# Temp variables to hold roll, pitch, and yaw values to make calculations cleaner
		roll = state_msg.roll
		pitch = state_msg.pitch
		yaw = state_msg.yaw

		# Convert to quaternion
		q_x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		q_y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
		q_z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
		q_w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		
		# Populate orientation part of message
		odom.pose.pose.orientation.x = q_x
		odom.pose.pose.orientation.y = q_y
		odom.pose.pose.orientation.z = q_z
		odom.pose.pose.orientation.w = q_w

		self.odom_pub.publish(odom)

	# Creates a path message to visualize the path through space the ROV took
	# ! Path stops in RVIZ after a while for some reason
	def path_callback(self, pose_msg):
		# Update header with newest stamp
		self.path.header = pose_msg.header

		# Create new pose stamped message to temporarily hold the pose
		# The path is a series of posestamped messages
		# pose = PoseStamped()

		# # Get header and pose data from odom message
		# pose.header = odom_msg.header
		# pose.pose = odom_msg.pose.pose

		# Append pose to the path's list of poses
		self.path.poses.append(pose_msg)

		# Publish the path
		self.path_pub.publish(self.path)



if __name__ == "__main__":
	# Initialize the node 
	rospy.init_node("state_viz", anonymous=True)

	# Visualizer object
	viz = rviz_visualizer()

	# Subscribe to the filtered robot state topic
	rospy.Subscriber("/imu_filtered_state", robot_state, viz.imu_state_callback)

	# Subsriber to our own odom topic to be used to create the path
	rospy.Subscriber("/robot_pose", PoseStamped, viz.path_callback)

	rospy.spin()