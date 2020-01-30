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
	//State vector [x, y, roll, pitch, yaw, x_dot, y_dot, roll_dot, pitch_dot, yaw_dot] 
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
		// Add all of the 10 state variables to the state vector as 0s
		for(int i = 0; i < 10; i++)
		{
			state_ << 0;
		}
		
		// Initialize the covariance matrix with 100 0s
		for(int i = 0; i < 10; i++)
		{
			cov_ << 0;
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

	// Callback function for the IMU messages
	ros::Subscriber imu_sub = n.subscribe("/vn100/imu/raw", 100, &EKF::IMUCallback, &ekf);

	// Callback function for the depth messages
	ros::Subscriber depth_sub = n.subscribe("/bar30/depth/raw", 100, &EKF::depthCallback, &ekf);

	// Callback function for the DVL messages
	ros::Subscriber dvl_sub = n.subscribe("/rti/body_velocity/raw", 100, &EKF::DVLCallback, &ekf);

	ros::spin();

	return 0;
}

