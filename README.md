# BueROV Kalman Filter (KF)

This repository contains code for a 6-DOF state estimation framework for the Robust Field Autonomy Lab's (RFAL) BlueROV2 platform. The state estimation is implemented as a Kalman filter that fuses sensor readings from the inertial measurement unit (IMU), doppler velocity logger (DVL), and pressure sensor. By combining these three sensors we are able to produce an accurate estimation of the ROV's 6-DOF state. This repostiory also contains various visualization tools that help show both the ROV's current state as well as its state over time. 

<p align='center'>
    <img src="/media/orientation_comparison.gif" alt="orientation_example" width="800"/>
</p>
<p align='center'>
    <img src="/media/velocity_comparison.gif" alt="velocity_example" width="800"/>
</p>


## Dependencies

- ROS (Only tested on Melodic so far)
- Custom BlueROV package from the lab's repository @ https://github.com/RobustFieldAutonomyLab/bluerov
- To download the package and make the custom messages available use the following commands
```
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/bluerov.git
cd ..
catkin_make
```
- C++ Eigen Matrix library (Included in the src folder)
- Python libraries opencv-python(cv2) and matplotlib


## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/roshea6/EKF.git
cd ..
catkin_make
```

The location of the repository may change place in the future to be part of the lab's repository.

## The System

The state estimation framework was designed to allow the lab's BlueROV platform to operate with low-drift dead-reckoning as it navigated through the water. The ROV configuration that the framework was initially designed for can be seen below but it should be fairly easy to add new sensors into the Kalman filter. 
<p align='center'>
    <img src="/media/Labeled_BlueROV.png" alt="BlueROV" width="800"/>
</p>

### Important Matrices

State Matrix: A 12x1 matrix containing the 6-DOF state as well as their derivatives. 
- The indices for each value in the state matrix are as follows
- [x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot] 
- [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

State Transition Matrix: 12x12 matrix used to update the state matrix every timestep
- 1 0 0 0 0 0 1 0 0 0 0 0 \
0 1 0 0 0 0 0 1 0 0 0 0 \
0 0 1 0 0 0 0 0 1 0 0 0 \
0 0 0 1 0 0 0 0 0 1 0 0 \
0 0 0 0 1 0 0 0 0 0 1 0 \
0 0 0 0 0 1 0 0 0 0 0 1 \
0 0 0 0 0 0 1 0 0 0 0 0 \
0 0 0 0 0 0 0 1 0 0 0 0 \
0 0 0 0 0 0 0 0 1 0 0 0 \
0 0 0 0 0 0 0 0 0 1 0 0 \
0 0 0 0 0 0 0 0 0 0 1 0 \
0 0 0 0 0 0 0 0 0 0 0 1 

Process Noise Covraiance Matrix: A 12x12 matrix for error associated with noise within the state prediction process

- Should be printed out at the beginning of the code by the print_state() function
- The values were tuned in groups with performance based on visual results from the visualization tools
- These values and this matrix in general can most likely be optimized. The values were only tuned until the performance for a specific sensor was deemed good enough so there is much room for improvement

Measurement Noise Matrices: Matrices of various sizes with one dedicated to each separate sensor
- Represent the noise associated with the sensor
- Starting values were taken from sensor datasheets and tuned from there until acceptable levels of performance were reached

## Running the Code

### Kalman Filter
To run just the Kalman Filter for the ROV use the following command:
```
rosrun EKF ekf
```
This will publish 3 topics of interest
- /6_DOF_state: A custom message with an array of the 6 values that make of the 6 DOF state of the robot
- /robot_pose: A PoseStamped message containing current robot pose with an associated timestamp
- /imu_filtered_state (MOST LIKELY GOING TO CHANGE): A custom message with the 12 values of the robot state mentioned earlier. Primarily used for debugging and tuning purposes

### Kalman Filter + Unfiltered vs Filtered State Visualizer (Might get removed)

To run the Kalman Filter + state visualization node use the following command:
```
roslaunch EKF ekf_visualizer.launch
```

This will play a bag file and show the unfiltered vs filtered output of the various sensors. Use it to tune values. Need to have a bag file to play!!

### Kalman Filter + BlueROV UI
To run the Kalman filter with the new UI for the BlueROV use the following command:
```
roslaunch EKF blue_rov_ekf_ui.launch
```

In addition to the KF this will launch and opencv window with both the M750 and M1200 Sonar images stitched together as well as a matplotlib window with graphed data of robot state over time.

<p align='center'>
    <img src="/media/blueROV_ui.gif" alt="BlueROV" width="800"/>
</p>


### Kalman Filter + RVIZ Visualization
To run the Kalman filter with the RVIZ visualization use the following command:
```
roslaunch EKF rviz_state_viz.launch
```
This will launch an RVIZ simulation that shows a 3 axis representation of the 6 DOF state of the robot over time.