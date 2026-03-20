# ME495 Sensing, Navigation and Machine Learning For Robotics
* Kyle Thompson
* Winter 2026
# Package List
This repository consists of several ROS packages
- `nuturtle_description` - Holds the XACRO/URDF description of turtlebot and launchfiles to open them in rviz.
- `turtlelib` - Geometry and transforms library with svg visualization
- `nusim` - Sets up the simulated environment and outputs for the turtlebot simulation
- `nuturtle_control` - Nodes for controlling the turtlebot including turtle_control and odometry
- `nuturtle_control_interfaces` - Custom service and message interfaces for nuturtle_control
- `nuslam` - EKF-SLAM implementation with circle landmark detection for the turtlebot

Homework 2 Circle Robot Video:

https://github.com/user-attachments/assets/24e11844-0caa-4965-83b5-c910ac92b1f7


Odom error:


x: 0.1308321191316738
y: 0.04392329900151747
z: 0.0


Homework 3 SLAM screenshot:

![Slam Image](nuslam/images/slam.png)