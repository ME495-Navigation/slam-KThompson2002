Simulation Run with:
`ros2 launch nuslam unknown_data_assoc.launch.xml`

Blue Odom error:
x: 0.044
y: 0.042
z: 0.0

Green Odom Error:
x: 0.001
y: 0.004
z: 0.0

Simulation Video:


https://github.com/user-attachments/assets/b0fe906f-7683-428a-ba8f-88fc58ca62ce



Real Robot Run with:
On Turtlebot:
`ros2 launch nuslam turtlebot_bringup.launch.xml`

On PC:
`ros2 launch nuslam pc_bringup.launch.xml`

Demo Run on Real Robot [Parameters still need to be tuned to fix position errors]:



https://github.com/user-attachments/assets/db032f0c-22ce-47be-a692-7124266b6f14



Changes which can be done to optimize real robot performance:

- Reduce initial Covariance
- Tune the threshold required for establishing a landmark
- Tune the time before resetting provisional time
