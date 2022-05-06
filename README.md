# apple_detection_rob545
Repo for rob545 final project of finding the center of an apple relative to the arm base.


This looks to be the RealSense repo needed for the camera:
https://github.com/IntelRealSense/realsense-ros

# Instructions

## Within Rviz 
Terminal 1:
`roslaunch apple_proxy pickApp.launch`

Terminal 2:
`rosrun apple_detection rob545_main.py`

## With the real UR5 and visualize it in Rviz
Terminal 1:
`roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=169.254.177.232`

Terminal 2:


Terminal 3:
`rosrun apple_detection rob545_main.py`


