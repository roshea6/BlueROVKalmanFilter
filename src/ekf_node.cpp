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

// Custom robot state message
#include <EKF/robot_state.h>


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

	ros::Publisher filtered_state_pub_ = n_.advertise<EKF::robot_state>("/filtered_state", 100);

	ros::Publisher unfiltered_state_pub_ = n_.advertise<EKF::robot_state>("/unfiltered_state", 100);

	// Variable to keep track of the previous time so we can find difference between timestamps
	float previous_timestamp_;

	// Variable to keep track of if the first sensor reading has been used to initilize the state matrix
	bool is_initialized_ = false;

	// Variable for storing most recent depth sensor message
	bar30_depth::Depth* depth_msg_ = nullptr;

	// Variable for storing the most recent DVL sensor message
	rti_dvl::DVL* DVL_msg_ = nullptr;


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

		// Initialize IMU measurement matrix
		H_IMU_ = MatrixXd(6, 12);

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

		for(int i = 0; i < 6; i++)
		{
			// ? Try setting these to just being the same value
			// Gyro noise for angular velocity
			if(i < 3)
			{
				R_IMU_(i, i) = .0035;
			}

			// Accelerometer noise for linear acceleration
			else
			{
				R_IMU_(i, i) = .14;
			}			
		}

		// ? Try using an identity matrix for this
		Q_ = MatrixXd(12, 12);

		// Initialize the process noise matrix
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
					Q_(i, j) = 1;
				}

				// Initialize derivate position variables along the diagonal to have a covariance of 1000
				// Rows 6 through 11 will be for the derivatives of the positions variables
				// x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot
				else if(i == j and i >= 6)
				{
					Q_(i, j) = 1;
				}
				
				// Fill the rest with 0's to start off
				else
				{
					Q_(i, j) = 0;
				}
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
			float yaw = 0; //atan2(accel_z, sqrt(accel_x*accel_x + accel_z*accel_z));

			// Update roll, pitch, and yaw values
			state_(3) = roll; 
			state_(4) = pitch;
			state_(5) = yaw;

			// Update derivatives of orientation variables
			// ! Change these back to positive posibbly
			state_(9) = -imu_msg.angular_velocity.x; // roll_dot
			state_(10) = -imu_msg.angular_velocity.y; // pitch_dot
			state_(11) = -imu_msg.angular_velocity.z; // yaw_dot

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

		// TODO: Update the state transition matrix F
		for(int i = 0; i < 6; i++)
		{
			// Update the off diagonal values for the derivatives of the position state variables
			// with the value of how much time has passed
			// Like before, the column index of a derivative of a position variable in the state matrix can be found by adding 6 to its index
			F_(i, i+6) = .005; // TODO: Change this to dt
		}
		

		predict(); // Predict the current state 

		// Check if the other messages are currently available to update the state

		// Check if a depth message is available
		if(depth_msg_ != nullptr)
		{
			// If a depth message is ready then update the state using the message
			depthKalmanUpdate();

			// Free the memory being using by the depth msg object
			delete depth_msg_;

			// Set the variable to nullptr so this doens't trigger again until we've received a new message
			depth_msg_ = nullptr;
		}

		// Check if a DVL message is available
		if(DVL_msg_ != nullptr)
		{
			// If a DVL message is ready then update the state using the message
			DVLKalmanUpdate();

			// Free the memory being using by the DVL msg object
			delete DVL_msg_;

			// Set the variable to nullptr so this doens't trigger again until we've received a new message
			DVL_msg_ = nullptr;
		}


		// Update measurement state mapping matrix H and sensor covariance matrix R
		IMUKalmanUpdate(imu_msg); // Use the data from the IMU to update the state


		// TODO: REMOVE THIS LATER IT'S JUST FOR TESTING VN STATE
		// Put quaternion values from message into easier to use values
		float q_x = imu_msg.orientation.x;
		float q_y = imu_msg.orientation.y;
		float q_z = imu_msg.orientation.z;
		float q_w = imu_msg.orientation.w;

		// Convert into roll, pitch, and yaw
		// These conversions are take directly from the VectorNav Quaternion math guide 
		// https://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/AN002.pdf?sfvrsn=19ee6b9_13
		float imu_yaw = 0; //atan2(2*(q_x*q_y + q_w*q_z), q_w*q_w - q_z*q_z - q_y*q_y + q_x*q_x);
		float imu_pitch = asin(-2*(q_x*q_z - q_y*q_w));
		float imu_roll = atan2(2*(q_y*q_z + q_x*q_w), q_w*q_w + q_z*q_z - q_y*q_y - q_x*q_x);

		// TODO: REMOVE THIS LATER IT'S JUST FOR TESTING

		// Create a robot state message
		EKF::robot_state state_msg;

		// Set the time for the state message
		state_msg.header.stamp = imu_msg.header.stamp;

		// Populate message with garbage data for now
		state_msg.x = state_(0);
		state_msg.y = state_(1);
		state_msg.z = state_(2);
		state_msg.roll = imu_roll;
		state_msg.pitch = imu_pitch;
		state_msg.yaw = imu_yaw;
		state_msg.x_dot = state_(6);
		state_msg.y_dot = state_(7);
		state_msg.z_dot = state_(8);
		state_msg.roll_dot = state_(9);
		state_msg.pitch_dot = state_(10);
		state_msg.yaw_dot = state_(11);

		// Publish message
		vn_state_pub_.publish(state_msg);


		// Publish filtered state after latest update
		// State message for filtered robot state
		EKF::robot_state filtered_state;

		// Set the timestamp for the message
		filtered_state.header.stamp = imu_msg.header.stamp;

		filtered_state.roll = state_[3]; 
		filtered_state.pitch = state_[4];
		filtered_state.yaw = state_[5];

		filtered_state_pub_.publish(filtered_state);

		// printState();
	}

	// Callback function for the depth messages from the bar30 Depth sensor
	void depthCallback(bar30_depth::Depth new_msg)
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
	// TODO: Figure out what part of the update step is causing the state and covariance matrix to all become NaN
	void IMUKalmanUpdate(const sensor_msgs::Imu imu_msg)
	{
		// State message for robot state
		EKF::robot_state uf_state;

		// Set the timestamp for the message
		uf_state.header.stamp = imu_msg.header.stamp;

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
		float yaw = 0; //atan2(accel_z, sqrt(accel_x*accel_x + accel_z*accel_z));

		// Update roll, pitch, and yaw values
		uf_state.roll = roll; 
		uf_state.pitch = pitch;
		uf_state.yaw = yaw;

		unfiltered_state_pub_.publish(uf_state);

		// Fill measurement vector z with roll, pitch, yaw, and derivatives values
		VectorXd z(6);

		// Put values into the vector
		z << roll, pitch, yaw, // roll, pitch, yaw
			 -imu_msg.angular_velocity.x, // roll_dot
			 -imu_msg.angular_velocity.y, // pitch_dot
			 -imu_msg.angular_velocity.z; // yaw_dot
		// ! Change these back to positive posibbly

		VectorXd y = z - H_IMU_ * state_; // Measurement error
		MatrixXd S = H_IMU_ * cov_ * (H_IMU_.transpose()) + R_IMU_;

		// TODO: Figure out why Kalman gain is always near 0
		MatrixXd K = cov_ * (H_IMU_.transpose()) * (S.inverse()); // Kalman gain

		// cout << "Kalman gain" << endl;
		// cout << K << endl;
		// cout << "\n";

		// Get new state
		state_ = state_ + K*y;

		// Create identity matrix and use it to update state covariance matrix
		MatrixXd I = MatrixXd::Identity(state_.size(), state_.size());

		cov_ = (I - K*H_IMU_)*cov_;

	}

	// Update the state based on the predicted state and the data from the Depth sensor
	void depthKalmanUpdate()
	{
		// TODO: Add the Kalman Filter update stuff for the Depth sensor
		// ! Message data to use is stored in depth_msg_
	}

	// Update the state based on the predicted state and the data from the DVL
	void DVLKalmanUpdate()
	{
		// TODO: Add the Kalman Filter update stuff for the DVL
		// ! Message data to us is stored in DVL_msg_
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

		// cout << "IMU H Matrix" << endl;
		// cout << H_IMU_ << endl;
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

