# apple_detection_rob545
Repo for rob545 final project of finding the center of an apple relative to the arm base.


This looks to be the RealSense repo needed for the camera:
https://github.com/IntelRealSense/realsense-ros

# Instructions

## Other required packages
Step in the same workspace: ~/your_ws/src/  
`git clone https://github.com/velasale/apple_proxy`


Realsense camera install:

```console
sudo apt-get install ros-melodic-realsense2-camera
```

```console
sudo apt-get install ros-melodic-realsense2-description
```

## Just visualization (Rviz) 
Terminal 1 (Motion planner - MoveIt & visualization - Rviz):  
`roslaunch apple_proxy pickApp.launch`

Terminal 2 (Our code):  
`rosrun apple_detection rob545_main.py`

## With the real UR5 and visualize it in Rviz
Terminal 1 (UR5 drivers):  
`roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=169.254.177.232`

Terminal 2 (Motion planner - MoveIt):  
`roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch`

Terminal 3 (Visualization - Rviz):  
`roslaunch ur5e_moveit_config moveit_rviz.launch config:=true`

Terminal 4 (Our code):  
`rosrun apple_detection rob545_main.py`



run command for camera:

```console
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true 
```