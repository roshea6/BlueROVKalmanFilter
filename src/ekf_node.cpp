// Extended Kalman Filter Node for BlueROV

#include <iostream>
#include "Eigen/Dense"
#include "ros/ros.h"

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

	// Robot state publisher
	ros::Publisher state_pub_ = n_.advertise<EKF::robot_state>("/raw_state", 100);

	// Variable to keep track of the previous time so we can find difference between timestamps
	float previous_timestamp_;

	// Variable to keep track of if the first sensor reading has been used to initilize the state matrix
	bool is_initialized_ = false;


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

		// Initialize the covariance matrix with 100 0s
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

		// ? Zero out for now 
		R_IMU_.setZero();

	}

	// Destuctor
	~extendedKF()
	{

	}

	// Callback function for IMU messages from the VN 100 IMU
	void IMUCallback(const sensor_msgs::Imu imu_msg)
	{
		// Check if the state has been initialized with a measurement yet
		if(!is_initialized_)
		{
			// Populate the state matrix with the first IMU readings
			// Take the values from the IMU message and put them in a quarternion vector
			Quaternionf quat;

			quat.x() = imu_msg.orientation.x;
			quat.y() = imu_msg.orientation.y;
			quat.z() = imu_msg.orientation.z;
			quat.w() = imu_msg.orientation.w;

			// Get the Euler angles from the quaternion
			Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

			// TODO: Fully Remove this later when I'm sure the math is correct
			// cout << "Euler angles:" << endl;
			// cout << "\t x: " << euler(0) << endl;
			// cout << "\t y: " << euler(1) << endl;
			// cout << "\t z: " << euler(2) << endl;
			// cout << endl;

			// Update roll, pitch, and yaw values
			state_(3) = euler(0); // roll
			state_(4) = euler(1); // pitch
			state_(5) = euler(2); // yaw

			// Update derivatives of orientation variables
			// ? Use anglular velocities from imu_msg
			state_(9) = imu_msg.angular_velocity.x; // roll_dot
			state_(10) = imu_msg.angular_velocity.y; // pitch_dot
			state_(11) = imu_msg.angular_velocity.z; // yaw_dot

			// Mark the state as initilized
			is_initialized_ = true;
			return;
		}

		// Calculate dt possibly
		// TODO: Figure out why IMU messages in rosbag don't have timestamps
		float dt = (imu_msg.header.stamp.toSec() - previous_timestamp_);
		
		// Update values of previous timestamp with value from latest message
		previous_timestamp_ = imu_msg.header.stamp.toSec();

		// TODO: Update the state transition matrix F
		for(int i = 0; i < 6; i++)
		{
			// Update the off diagonal values for the derivatives of the position state variables
			// with the value of how much time has passed
			// Like before, the column index of a derivative of a position variable in the state matrix can be found by adding 6 to its index
			F_(i, i+6) = .005; // TODO: Change this from 1 to dt
		}
		
		// TODO: Figure out how to update process noise matrix Q
		Q_ = MatrixXd(12, 12);

		Q_.setZero();

		predict(); // Predict the current state 


		// Update measurement state mapping matrix H and sensor covariance matrix R
		// IMUKalmanUpdate(imu_msg); // Use the data from the IMU to update the state

		// Create a robot state message
		EKF::robot_state state_msg;

		// Set the time for the state message
		state_msg.header.stamp = ros::Time::now();

		// Populate message with garbage data for now
		state_msg.x = state_(0);
		state_msg.y = state_(1);
		state_msg.z = state_(2);
		state_msg.roll = state_(3);
		state_msg.pitch = state_(4);
		state_msg.yaw = state_(5);
		state_msg.x_dot = state_(6);
		state_msg.y_dot = state_(7);
		state_msg.z_dot = state_(8);
		state_msg.roll_dot = state_(9);
		state_msg.pitch_dot = state_(10);
		state_msg.yaw_dot = state_(11);

		// Publish message
		state_pub_.publish(state_msg);
	}

	// Callback function for the depth messages from the bar30 Depth sensor
	void depthCallback(const bar30_depth::Depth depth_msg)
	{
		// cout << "Depth message:" << endl;
		// cout << depth_msg << endl;

		// Calculate dt

		// Update the state transition matrix F and process noise matrix Q
		predict(); // Predict the current state 

		// Update measurement state mapping matrix H and sensor covariance matrix R
		depthKalmanUpdate(depth_msg); // Use the data from the IMU to update the state

	}

	// Callback function for the doppler velocity logger (DVL) messages from the **** DVL
	void DVLCallback(const rti_dvl::DVL dvl_msg)
	{
		// cout << "DVL message:" << endl;
		// cout << dvl_msg << endl;

		// Calculate dt

		// Update the state transition matrix F and process noise matrix Q
		predict(); // Predict the current state 

		// Update measurement state mapping matrix H and sensor covariance matrix R
		DVLKalmanUpdate(dvl_msg); // Use the data from the IMU to update the state

	}
	

	// Predict the current state based on the previous state
	void predict() 
	{
		cout << "Start predict" << endl;
		printState();

		// State prediction
		state_ = F_ * state_;

		// Covariance matrix prediction
		cov_ = F_*cov_*F_.transpose() + Q_;

		cout << "End predict" << endl;
		printState();
	}

	// Update the state based on the predicted state and the data from the IMU
	void IMUKalmanUpdate(const sensor_msgs::Imu imu_msg)
	{
		cout << "Start update" << endl;
		printState();

		// TODO: Add the Kalman Filter update stuff for the IMU
		// Take the values from the IMU message and put them in a quarternion vector
		Quaternionf quat;

		quat.x() = imu_msg.orientation.x;
		quat.y() = imu_msg.orientation.y;
		quat.z() = imu_msg.orientation.z;
		quat.w() = imu_msg.orientation.w;

		// Get the Euler angles from the quaternion
		Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

		// Fill measurement vector z with roll, pitch, yaw, and derivatives values
		VectorXd z(6);

		// Put values into the vector
		z << euler(0), euler(1), euler(2), // roll, pitch, yaw
			 imu_msg.angular_velocity.x, // roll_dot
			 imu_msg.angular_velocity.y, // pitch_dot
			 imu_msg.angular_velocity.z; // yaw_dot

		VectorXd y = z - H_IMU_ * state_; // Measurement error

		// cout << "y" << endl;
		// cout << y << endl;

		MatrixXd S = H_IMU_ * cov_ * H_IMU_.transpose() + R_IMU_;
		MatrixXd K = cov_ * H_IMU_.transpose() * S.inverse(); // Kalman gain

		// Get new state
		state_ = state_ + K*y;

		// Create identity matrix and use it to update state covariance matrix
		MatrixXd I = MatrixXd::Identity(state_.size(), state_.size());

		cov_ = (I - K*H_IMU_)*cov_;

		cout << "End update" << endl;
		printState();

	}

	// Update the state based on the predicted state and the data from the Depth sensor
	void depthKalmanUpdate(const bar30_depth::Depth depth_msg)
	{
		// TODO: Add the Kalman Filter update stuff for the Depth sensor
	}

	// Update the state based on the predicted state and the data from the DVL
	void DVLKalmanUpdate(const rti_dvl::DVL dvl_msg)
	{
		// TODO: Add the Kalman Filter update stuff for the DVL
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

