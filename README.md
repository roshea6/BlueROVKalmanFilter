# BueROV Extended Kalman Filter (EKF)

This repository contains code for a 6-DOF state estimation framework for the Robust Field Autonomy Lab's (RFAL) BlueROV2 platform. The state estimation is implemented as an Extended Kalman filter that fuses sensor readings from the inertial measurement unit (IMU), doppler velocity logger (DVL), and pressure sensor. By combining these three sensors we are able to produce an accurate estimation of the ROV's 6-DOF state. This repostiory also contains various visualization tools that help show both the ROV's current state as well as its state over time. 

<p align='center'>
    <img src="/media/orientation_comparison.gif" alt="orientation_example" width="800"/>
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

