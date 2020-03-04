// Extended Kalman Filter Node for BlueROV

#include <iostream>
#include "Eigen/Dense"
#include "ros/ros.h"
#include <math.h>

// IMU messages
#include <sensor_msgs/Imu.h>

// Depth sensor messages
#include <bar30_depth/Depth.h>

// DVL messages
#include <rti_dvl/DVL.h>

// Custom robot state messages
#include <EKF/robot_state.h>
#include <EKF/robot_6_DOF_state.h>

// Posestamped messages
#include <geometry_msgs/PoseStamped.h>


using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Quaternionf;

class extendedKF {
private:
	//State vector [x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot] 
	// Numbered    [0, 1, 2, 3,    4,     5,   6,     7,     8,     9,        10,        11]
	VectorXd state_;
	
	// State covariance matrix
	MatrixXd cov_;

	// state transition matrix
	MatrixXd F_;

	// process covariance matrix
	MatrixXd Q_;

	// IMU measurement matrix
	MatrixXd H_IMU_;

	// Depth measurement matrx
	MatrixXd H_depth_;

	// DVL measurement matrix
	MatrixXd H_DVL_;

	// IMU measurement covariance matrix
	MatrixXd R_IMU_;

	// Depth measurement covariance matrix
	MatrixXd R_depth_;

	// DVL measurement covariance matrix
	MatrixXd R_DVL_;

	// Node handle for creating publishers within the class
	ros::NodeHandle n_;

	// VectorNav Robot state publisher
	ros::Publisher vn_state_pub_ = n_.advertise<EKF::robot_state>("/vn_state", 100);

	// IMU Filtered state publisher
	ros::Publisher imu_filtered_state_pub_ = n_.advertise<EKF::robot_state>("/imu_filtered_state", 100);

	// IMU Unfiltered state publisher
	ros::Publisher imu_unfiltered_state_pub_ = n_.advertise<EKF::robot_state>("/imu_unfiltered_state", 100);

	// DVL Filtered state publisher
	ros::Publisher dvl_filtered_state_pub_ = n_.advertise<EKF::robot_state>("/dvl_filtered_state", 100);

	// DVL Unfiltered state publisher
	ros::Publisher dvl_unfiltered_state_pub_ = n_.advertise<EKF::robot_state>("/dvl_unfiltered_state", 100);

	// DVL Filtered state publisher
	ros::Publisher depth_filtered_state_pub_ = n_.advertise<EKF::robot_state>("/depth_filtered_state", 100);

	// DVL Unfiltered state publisher
	ros::Publisher depth_unfiltered_state_pub_ = n_.advertise<EKF::robot_state>("/depth_unfiltered_state", 100);

	// State publisher for the 6 DOF state (x, y, z, roll, pitch, yaw)
	ros::Publisher DOF_6_state_pub_ = n_.advertise<EKF::robot_6_DOF_state>("/6_DOF_state", 100);

	// Pose publisher for robot
	ros::Publisher robot_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/robot_pose", 100);

	// Variable to keep track of the previous time so we can find difference between timestamps
	float previous_timestamp_;

	// Variable to keep track of if the first sensor reading has been used to initilize the state matrix
	bool is_initialized_ = false;

	// Variable for storing most recent depth sensor message
	bar30_depth::Depth* depth_msg_ = nullptr;

	// Variable for storing the most recent DVL sensor message
	rti_dvl::DVL* DVL_msg_ = nullptr;

	// Define the max robot velocity. Used to filter out noisy values from DVL
	const float MAX_VEL = 5.0;


public:
	// Constructor 
	extendedKF()
	{
		// Declare state vector to be of size 12
		state_ = VectorXd(12);

		// Add all of the 10 state variables to the state vector as 0s
		for(int i = 0; i < 12; i++)
		{
			state_(i) = 0;
		}
		

		// Declare state covariance matrix as a 12x12 matrix
		cov_ = MatrixXd(12, 12);

		// Initialize the covariance matrix
		for(int i = 0; i < 12; i++)
		{
			for(int j = 0; j < 12; j++)
			{
				// Initialize position variables along the diagonal to have a covariance of 1
				// The main diagonal can be represented by when i is equal to j
				// We use i < 6 because rows 0 through 5 will be for the position variables
				// x, y, z, roll, pitch, yaw 
				if(i == j and i < 6)
				{
					cov_(i, j) = 1;
				}

				// Initialize derivate position variables along the diagonal to have a covariance of 1000
				// Rows 6 through 11 will be for the derivatives of the positions variables
				// x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot
				else if(i == j and i >= 6)
				{
					cov_(i, j) = 1000;
				}
				
				// Fill the rest with 0's to start off
				else
				{
					cov_(i, j) = 0;
				}
			}
		}

		// Declare state transition matrix F
		F_ = MatrixXd(12, 12);

		// Initialize F with 1's along the diagonal and 1's in the corresponding column
		// of position variable's derivatives
		for(int i = 0; i < 12; i++)
		{
			for(int j = 0; j < 12; j++)
			{
				// Set all locations on the main diagonal to 1
				if(i == j)
				{
					F_(i, j) = 1;
				}

				// For each row that represents a position value (rows 0 to 5) set the rowth + 5 value to 1
				// The row index + 6 is equal to the derivative of that position variable in the state matrix
				else if((j == i + 6) and (i < 6))
				{
					F_(i, j) = 1;
				}

				// Set all other values to 0
				else
				{
					F_(i, j) = 0;
				}		
			}
		}

		// ############## IMU SPECIFIC MATRICES ####################

		// Initialize IMU measurement mapping matrix
		H_IMU_ = MatrixXd(6, 12); //6x12 because we need to get roll, pitch, yaw and their derivatives from the state matrix

		// Zero out the whole matrix to start
		H_IMU_.setZero();

		for(int i = 0; i < 6; i++)
		{
			// Fill in roll, pitch and yaw values
			if(i < 3)
			{
				H_IMU_(i, i+3) = 1; // Use +3 to fill in places for roll, pitch, yaw instead of x, y, z
			}

			// Fill in roll_dot, pitch_dot and yaw_dot values
			else
			{
				H_IMU_(i, i+6) = 1; // Use +6 to fill in places for roll_dot, pithch_dot, yaw_dot instead of roll, pitch, yaw
			}	
		}

		// TODO: Initialize the IMU measurement covairance matrix
		R_IMU_ = MatrixXd(6, 6);

		// Set everything in the matrix to 0 because we only have to initialize the main diagnonal
		R_IMU_.setZero();

		for(int i = 0; i < 6; i++)
		{
			// ? Try setting these to just being the same value
			// Accelerometer noise for linear acceleration which is used to find orientation
			if(i < 3)
			{
				R_IMU_(i, i) = .15;
			}

			// Gyroscope noise used to find angular velocity
			else
			{
				R_IMU_(i, i) = .01; //.035
			}			
		}
		// ############## IMU SPECIFIC MATRICES ####################

		// ############## DVL SPECIFIC MATRICES ####################
		// Initialize DVL Measurement mapping matrix H_DVL_
		H_DVL_ = MatrixXd(3, 12); // 3x12 because we need to get x_dot, y_dot, and z_dot from the state matrix

		// Start the matrix as zeros
		H_DVL_.setZero();

		for(int i = 0; i < 3; i++)
		{
			// Set the 6th, 7th, and 8th index of rows 1, 2, and 3 respectively equal to 1
			H_DVL_(i, i+6) = 1;
		}

		// Initialize the DVL measurement noise matrix R_DVL_
		R_DVL_ = MatrixXd(3, 3); // 3x3 because we need noise for the x_dot, y_dot, and z_dot measurements

		// Start with the matrix as zeros
		R_DVL_.setZero();

		// Values taken from RTI site but need to double check
		// Gave range of .05 m/s to .2m/s
		for(int i = 0; i < 3; i++)
		{
			// TODO: tune this value
			R_DVL_(i, i) = .4;
		}
		// ############## DVL SPECIFIC MATRICES ####################

		// ############## DEPTH SPECIFIC MATRICES ####################
		// Initialize the Depth sensor measurement mammping matrix H
		H_depth_= MatrixXd(1, 12); // 1x12 because we only need to get the z value from the state

		// Start the matrix as zeros
		H_depth_.setZero();

		// Put a 1 at the index that corresponds to the z state variables (AKA depth)
		H_depth_(0, 2) = 1;

		// Initialize the Depth sensor measurement noise matrix R
		R_depth_ = MatrixXd(1, 1); //1x1 because we only need the noise value for the depth measurement

		// Initialize with the value in the sensors documentation
		// https://bluerobotics.com/store/sensors-sonars-cameras/sensors/bar30-sensor-r1/
		R_depth_(0, 0) = 1; //2.04; // ! This seems very high
		// ############## DEPTH SPECIFIC MATRICES ####################


		// Initialize process noise covariance matrix Q
		Q_ = MatrixXd(12, 12);

		Q_.setZero();

		// Initialize the process noise matrix
		for(int i = 0; i < 12; i++)
		{
			for(int j = 0; j < 12; j++)
			{
				// TODO: Can probably make this a switch statement
				// Initialize position variables along the diagonal to have a covariance of 1
				// The main diagonal can be represented by when i is equal to j
				// We use i < 6 because rows 0 through 5 will be for the position variables
				// x, y, z, roll, pitch, yaw 
				if(i == j and i < 2)
				{
					Q_(i, j) = 0;
				}

				// Process noise for z value acquired from depth sensor
				if(i == j and i == 2)
				{
					// TODO: tune this
					Q_(i, j) = .0001;
				}

				// IMU Process noise initialization for Roll, pitch, and yaw
				else if(i == j and i < 6 and i >= 3)
				{
					Q_(i, j) = .00001;
				}

				// DVL Process noise initialization for x_dot, y_dot, z_dot
				else if(i == j and i >=6 and i < 10)
				{
					// TODO: Tune this value
					Q_(i, j) = .00001;
				}

				// IMU process noise initialization for roll_dot, pitch_dot, yaw_dot
				else if(i == j and i >= 9 and i < 12)
				{
					Q_(i, j) = .00001;
				}
				
				// Fill the rest with 0's to start off
				// else
				// {
				// 	Q_(i, j) = 0;
				// }
			}
		}

		cout << "Process covariance" << endl;
		cout << Q_ << endl;
	}

	// Destuctor
	~extendedKF()
	{

	}

	// Callback function for IMU messages from the VN 100 IMU
	void IMUCallback(const sensor_msgs::Imu imu_msg)
	{
		// Check if the state has been initialized with a measurement yet
		// ! Might be a good idea to just remove this and leave the initial state as all 0's
		if(!is_initialized_)
		{
			// Calclate roll, pitch and yaw from the linear acceleration values
			// Temporary variables just to make calculations cleaner
			float accel_x = -imu_msg.linear_acceleration.x;
			float accel_y = -imu_msg.linear_acceleration.y;
			float accel_z = -imu_msg.linear_acceleration.z;

			// Find roll in radians
			float roll = atan2(accel_y, accel_z);

			// Find pitch in radians
			float pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z));

			// Find yaw in radians
			float yaw = atan2(accel_z, sqrt(accel_x*accel_x + accel_z*accel_z));

			// ! Pick nice starting depth for graph
			state_(2) = 0;

			// Update roll, pitch, and yaw values
			state_(3) = roll; 
			state_(4) = pitch;
			state_(5) = yaw;

			// Update derivatives of orientation variables
			// ! Change these back to positive posibbly
			state_(9) = imu_msg.angular_velocity.x; // roll_dot
			state_(10) = imu_msg.angular_velocity.y; // pitch_dot
			state_(11) = imu_msg.angular_velocity.z; // yaw_dot // ! Change this back from 0

			// Mark the state as initilized
			is_initialized_ = true;
			return;
		}

		// Calculate dt possibly
		// TODO: Figure out why IMU messages in rosbag don't have timestamps
		// TODO: dt for Kalman should be fixed
		// ? Try making dt a constant of 1/200 to match rate for IMU publishing frequency
		float dt = (imu_msg.header.stamp.toSec() - previous_timestamp_);
		
		// Update values of previous timestamp with value from latest message
		previous_timestamp_ = imu_msg.header.stamp.toSec();

		// Update the state transition matrix with the timestamp values
		for(int i = 0; i < 6; i++)
		{
			// Update the off diagonal values for the derivatives of the position state variables
			// with the value of how much time has passed
			// Like before, the column index of a derivative of a position variable in the state matrix can be found by adding 6 to its index
			F_(i, i+6) = .005; //1/200 to match the frequency of the IMU
		}
		

		predict(); // Predict the current state 

		// Check if the other messages are currently available to update the state

		// Check if a depth message is available
		if(depth_msg_ != nullptr)
		{
			// TODO: Publish unfiltered depth here
			// Create state message to hold depth data
			EKF::robot_state uf_depth;

			// Set time stamp for message
			uf_depth.header.stamp = ros::Time::now();

			// TODO: Figure out if this should be negative
			uf_depth.z = depth_msg_->depth;

			depth_unfiltered_state_pub_.publish(uf_depth);

			// If a depth message is ready then update the state using the message
			depthKalmanUpdate();

			// TODO: Publish filtered depth here
			// Create state message to hold depth data
			EKF::robot_state filt_depth;

			// Set timestamp for header
			filt_depth.header.stamp = ros::Time::now();

			// TODO: Figure out if this should be negative
			filt_depth.z = state_[2];

			depth_filtered_state_pub_.publish(filt_depth);

			// Free the memory being using by the depth msg object
			delete depth_msg_;

			// Set the variable to nullptr so this doens't trigger again until we've received a new message
			depth_msg_ = nullptr;
		}

		// Check if a DVL message is available
		if(DVL_msg_ != nullptr)
		{
			// TODO: Remove this later. Just for testing
			EKF::robot_state uf_dvl;

			// Temp variables to make code look cleaner
			float x_vel = DVL_msg_->velocity.x;
			float y_vel = DVL_msg_->velocity.y;
			float z_vel = DVL_msg_->velocity.z;

			uf_dvl.header.stamp = ros::Time::now(); //DVL_msg_->header.stamp;
			uf_dvl.x_dot = x_vel;
			uf_dvl.y_dot = y_vel;
			uf_dvl.z_dot = z_vel;

			dvl_unfiltered_state_pub_.publish(uf_dvl);

			// If a DVL message is ready then update the state using the message
			DVLKalmanUpdate();

			// TODO: Remove this later. Just for testing
			EKF::robot_state filt_dvl;

			filt_dvl.header.stamp = ros::Time::now(); //DVL_msg_->header.stamp;
			filt_dvl.x_dot = state_(6);
			filt_dvl.y_dot = state_(7);
			filt_dvl.z_dot = state_(8);

			dvl_filtered_state_pub_.publish(filt_dvl);

			// Free the memory being using by the DVL msg object
			delete DVL_msg_;

			// Set the variable to nullptr so this doens't trigger again until we've received a new message
			DVL_msg_ = nullptr;
		}


		// Update measurement state mapping matrix H and sensor covariance matrix R
		IMUKalmanUpdate(imu_msg); // Use the data from the IMU to update the state


		// // TODO: REMOVE THIS LATER IT'S JUST FOR TESTING VN STATE
		// // Put quaternion values from message into easier to use values
		// float q_x = imu_msg.orientation.x;
		// float q_y = imu_msg.orientation.y;
		// float q_z = imu_msg.orientation.z;
		// float q_w = imu_msg.orientation.w;

		// // Convert into roll, pitch, and yaw
		// // These conversions are take directly from the VectorNav Quaternion math guide 
		// // https://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/AN002.pdf?sfvrsn=19ee6b9_13
		// float imu_yaw = atan2(2*(q_x*q_y + q_w*q_z), q_w*q_w - q_z*q_z - q_y*q_y + q_x*q_x);
		// float imu_pitch = asin(-2*(q_x*q_z - q_y*q_w));
		// float imu_roll = atan2(2*(q_y*q_z + q_x*q_w), q_w*q_w + q_z*q_z - q_y*q_y - q_x*q_x);

		// // TODO: REMOVE THIS LATER IT'S JUST FOR TESTING

		// // Create a robot state message
		// EKF::robot_state state_msg;

		// // Set the time for the state message
		// state_msg.header.stamp = imu_msg.header.stamp;

		// // Populate message with state data
		// state_msg.x = state_(0);
		// state_msg.y = state_(1);
		// state_msg.z = state_(2);
		// state_msg.roll = imu_roll;
		// state_msg.pitch = imu_pitch;
		// state_msg.yaw = imu_yaw;
		// state_msg.x_dot = state_(6);
		// state_msg.y_dot = state_(7);
		// state_msg.z_dot = state_(8);
		// state_msg.roll_dot = state_(9);
		// state_msg.pitch_dot = state_(10);
		// state_msg.yaw_dot = state_(11);

		// // Publish message
		// vn_state_pub_.publish(state_msg);

		// State message for filtered 6 DOF state
		EKF::robot_6_DOF_state state_6_DOF;

		// Set the timestamp for the message
		state_6_DOF.header.stamp = ros::Time::now();

		// Push the first 6 values of the state_ array into the array message
		// These first 6 values correspond to x, y, z, roll, pitch, and yaw respectively
		for(int i = 0; i < 6; i++)
		{
			state_6_DOF.state_data.data.push_back(state_[i]);
		}
		
		DOF_6_state_pub_.publish(state_6_DOF);

		// Publish the robot pose
		publishRobotPose();

		// Publish filtered state after latest update
		// State message for filtered robot state
		EKF::robot_state filtered_state;

		// Set the timestamp for the message
		filtered_state.header.stamp = ros::Time::now();  //imu_msg.header.stamp

		// Populate message with state data
		filtered_state.x = state_[0];
		filtered_state.y = state_[1];
		filtered_state.z = state_[2];
		filtered_state.roll = state_[3]; 
		filtered_state.pitch = state_[4];
		filtered_state.yaw = state_[5];
		filtered_state.x_dot = state_[6];
		filtered_state.y_dot = state_[7];
		filtered_state.z_dot = state_[8];

		imu_filtered_state_pub_.publish(filtered_state);

		// printState();
	}

	// Callback function for the depth messages from the bar30 Depth sensor
	void depthCallback(const bar30_depth::Depth new_msg)
	{
		// Allocate the space for the new depth message to be saved
		depth_msg_ = new bar30_depth::Depth;

		// Populate the saved message's data with the data from the new depth message
		*depth_msg_ = new_msg;
	}

	// Callback function for the doppler velocity logger (DVL) messages from the **** DVL
	void DVLCallback(const rti_dvl::DVL new_msg)
	{
		// Allocate the space for a new DVL message to be saved
		DVL_msg_ = new rti_dvl::DVL;

		// Populate the saved message's data with the data from the new DVL message
		*DVL_msg_ = new_msg;
	}
	

	// Predict the current state based on the previous state
	void predict() 
	{
		// State prediction
		state_ = F_ * state_;

		// Covariance matrix prediction
		cov_ = F_*cov_*(F_.transpose()) + Q_;
	}

	// Update the state based on the predicted state and the data from the IMU
	void IMUKalmanUpdate(const sensor_msgs::Imu imu_msg)
	{
		// State message for robot state
		EKF::robot_state uf_state;

		// Set the timestamp for the message
		uf_state.header.stamp = ros::Time::now(); //imu_msg.header.stamp;

		// Calclate roll, pitch and yaw from the linear acceleration values
		// Temporary variables just to make calculations cleaner
		float accel_x = -imu_msg.linear_acceleration.x;
		float accel_y = -imu_msg.linear_acceleration.y;
		float accel_z = -imu_msg.linear_acceleration.z;

		// Find roll in radians
		float roll = atan2(accel_y, accel_z);

		// Find pitch in radians
		float pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z));

		// Find yaw in radians
		float yaw = atan2(accel_z, sqrt(accel_x*accel_x + accel_z*accel_z));

		// Update roll, pitch, and yaw values
		uf_state.roll = roll; 
		uf_state.pitch = pitch;
		uf_state.yaw = yaw;

		imu_unfiltered_state_pub_.publish(uf_state);

		// Fill measurement vector z with roll, pitch, yaw, and derivatives values
		VectorXd z_meas(6);

		// Put values into the vector
		z_meas << roll, pitch, yaw, // roll, pitch, yaw
			 imu_msg.angular_velocity.x, // roll_dot
			 imu_msg.angular_velocity.y, // pitch_dot
			 imu_msg.angular_velocity.z; // yaw_dot // ! Change this back from 0

		VectorXd z_pred = H_IMU_ *state_;

		VectorXd y = z_meas - z_pred; // Measurement error

		MatrixXd S = H_IMU_ * cov_ * (H_IMU_.transpose()) + R_IMU_;

		MatrixXd K = cov_ * (H_IMU_.transpose()) * (S.inverse()); // Kalman gain

		// Get new state
		state_ = state_ + K*y;

		// Create identity matrix and use it to update state covariance matrix
		MatrixXd I = MatrixXd::Identity(state_.size(), state_.size());

		cov_ = (I - K*H_IMU_)*cov_;

	}

	// Update the state based on the predicted state and the data from the Depth sensor
	void depthKalmanUpdate()
	{
		// ! Message data to use is stored in depth_msg_
		// Fill measurement vector z with roll, pitch, yaw, and derivatives values
		VectorXd z_meas(1);

		// Put values into the vector
		z_meas << depth_msg_->depth;

		VectorXd z_pred = H_depth_ *state_;

		VectorXd y = z_meas - z_pred; // Measurement error

		MatrixXd S = H_depth_ * cov_ * (H_depth_.transpose()) + R_depth_;

		MatrixXd K = cov_ * (H_depth_.transpose()) * (S.inverse()); // Kalman gain

		// Get new state
		state_ = state_ + K*y;

		// Create identity matrix and use it to update state covariance matrix
		MatrixXd I = MatrixXd::Identity(state_.size(), state_.size());

		cov_ = (I - K*H_depth_)*cov_;
	}

	// Update the state based on the predicted state and the data from the DVL
	void DVLKalmanUpdate()
	{
		// Fill measurement vector z with roll, pitch, yaw, and derivatives values
		VectorXd z_meas(3);

		// Temp variables to make code look cleaner
		float x_vel = DVL_msg_->velocity.x;
		float y_vel = DVL_msg_->velocity.y;
		float z_vel = DVL_msg_->velocity.z;

		// TODO: make these real filters
		if(abs(x_vel) > MAX_VEL)
		{
			x_vel = 0;
		}
		if(abs(y_vel) > MAX_VEL)
		{
			y_vel = 0;
		}
		if(abs(z_vel) > MAX_VEL)
		{
			z_vel = 0;
		}

		// Put values into the vector
		z_meas << x_vel, y_vel, z_vel;

		VectorXd z_pred = H_DVL_ *state_;

		VectorXd y = z_meas - z_pred; // Measurement error

		MatrixXd S = H_DVL_ * cov_ * (H_DVL_.transpose()) + R_DVL_;

		MatrixXd K = cov_ * (H_DVL_.transpose()) * (S.inverse()); // Kalman gain

		// Get new state
		state_ = state_ + K*y;

		// Create identity matrix and use it to update state covariance matrix
		MatrixXd I = MatrixXd::Identity(state_.size(), state_.size());

		cov_ = (I - K*H_DVL_)*cov_;
	}

	// Calculates the robots current pose in x, y, z for translation and quaternion for orientation
	// then publishes it
	void publishRobotPose()
	{
		// Blank pose stamped message
		geometry_msgs::PoseStamped robot_pose;

		// Add the timestamp to the message
		robot_pose.header.stamp = ros::Time::now();
		
		// ? Add parent frame to the header
		robot_pose.header.frame_id = "map";

		// Get the position of the robot from the state matrix
		robot_pose.pose.position.x = state_[0];
		robot_pose.pose.position.y = state_[1];
		robot_pose.pose.position.z = -state_[2]; // Negative because positive depth = below the surface

		// Calcualate the robot orientation in quaternion
		// Temp variables to make calculations cleaner
		float roll = state_[3];
		float pitch = state_[4];
		float yaw = state_[5];

	 	// Convert to quaternion
		float q_x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
		float q_y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
		float q_z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
		float q_w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

		// Populate the orientation part of the message
		robot_pose.pose.orientation.x = q_x;
		robot_pose.pose.orientation.y = q_y;
		robot_pose.pose.orientation.z = q_z;
		robot_pose.pose.orientation.w = q_w;
		
		robot_pose_pub_.publish(robot_pose);
	}

	// Prints out the current state matrix and covariance state matrix for debugging purposes
	void printState()
	{
		cout << "State:" << endl;
		cout << state_ << endl;
		cout << "\n";

		cout << "Covariance:" << endl;
		cout << cov_ << endl;
		cout << "\n";

		// cout << "State Transition" << endl;
		// cout << F_ << endl;
		// cout << "\n";

		// cout << "DVL H Matrix" << endl;
		// cout << H_DVL_ << endl;
		// cout << "\n";

		// cout << "DVL R Matrix" << endl;
		// cout << R_DVL_ << endl;
		// cout << "\n";

		// cout << "Q Matrix" << endl;
		// cout << Q_ << endl;
		// cout << "\n";
	}

};


// Main function that sets up the subscriber to each of the topics
int main(int argc, char **argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "EKF");

	// Create the node handle to communicate with ROS
	ros::NodeHandle n;

	// Create and instance of the extended Kalman Filter Object
	extendedKF ekf;

	// Display the state vector and covariance matrix.
	// TODO: Remove later. Just for debugging
	ekf.printState();

	// Callback function for the IMU messages
	ros::Subscriber imu_sub = n.subscribe("/vn100/imu/raw", 100, &extendedKF::IMUCallback, &ekf);

	// Callback function for the depth messages
	ros::Subscriber depth_sub = n.subscribe("/bar30/depth/raw", 100, &extendedKF::depthCallback, &ekf);

	// Callback function for the DVL messages
	ros::Subscriber dvl_sub = n.subscribe("/rti/body_velocity/raw", 100, &extendedKF::DVLCallback, &ekf);

	ros::spin();

	return 0;
}

