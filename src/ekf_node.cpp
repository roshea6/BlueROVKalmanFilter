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


using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKF {
private:
	//State vector [x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot] 
	VectorXd state_;
	
	// State covariance matrix
	MatrixXd cov_;

	// state transition matrix
	MatrixXd F_;

	// process covariance matrix
	MatrixXd Q_;

	// measurement matrix
	MatrixXd H_;

	// measurement covariance matrix
	MatrixXd R_;


public:
	// Constructor 
	EKF()
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
	}

	// Destuctor
	~EKF()
	{

	}

	// Callback function for IMU messages from the VN 100 IMU
	void IMUCallback(const sensor_msgs::Imu imu_msg)
	{
		cout << "IMU message:" << endl;
		// cout << imu_msg << endl;

		// Calculate dt possibly by saving timestamp from previous message

		// Update the state transition matrix F and process noise matrix Q
		predict(); // Predict the current state 

		// Update measurement state mapping matrix H and sensor covariance matrix R
		IMUKalmanUpdate(imu_msg); // Use the data from the IMU to update the state

	}

	// Callback function for the depth messages from the bar30 Depth sensor
	void depthCallback(const bar30_depth::Depth depth_msg)
	{
		cout << "Depth message:" << endl;
		cout << depth_msg << endl;

		// Calculate dt

		// Update the state transition matrix F and process noise matrix Q
		predict(); // Predict the current state 

		// Update measurement state mapping matrix H and sensor covariance matrix R
		depthKalmanUpdate(depth_msg); // Use the data from the IMU to update the state

	}

	// Callback function for the doppler velocity logger (DVL) messages from the **** DVL
	void DVLCallback(const rti_dvl::DVL dvl_msg)
	{
		cout << "DVL message:" << endl;
		cout << dvl_msg << endl;

		// Calculate dt

		// Update the state transition matrix F and process noise matrix Q
		predict(); // Predict the current state 

		// Update measurement state mapping matrix H and sensor covariance matrix R
		DVLKalmanUpdate(dvl_msg); // Use the data from the IMU to update the state

	}

	// Predict the current state based on the previous state
	void predict() 
	{
		// TODO: Add the Kalman Filter prediction step
	}

	// Update the state based on the predicted state and the data from the IMU
	void IMUKalmanUpdate(const sensor_msgs::Imu imu_msg)
	{
		// TODO: Add the Kalman Filter update stuff for the IMU
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
	EKF ekf;

	ekf.printState();

	// Callback function for the IMU messages
	ros::Subscriber imu_sub = n.subscribe("/vn100/imu/raw", 100, &EKF::IMUCallback, &ekf);

	// Callback function for the depth messages
	ros::Subscriber depth_sub = n.subscribe("/bar30/depth/raw", 100, &EKF::depthCallback, &ekf);

	// Callback function for the DVL messages
	ros::Subscriber dvl_sub = n.subscribe("/rti/body_velocity/raw", 100, &EKF::DVLCallback, &ekf);

	ros::spin();

	return 0;
}

