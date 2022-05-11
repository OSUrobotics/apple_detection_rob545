# apple_detection_rob545
Repo for rob545 final project of finding the center of an apple relative to the arm base.


This looks to be the RealSense repo needed for the camera:
https://github.com/IntelRealSense/realsense-ros

# Instructions

## Other required packages
Step in the same workspace: ~/your_ws/src/  
```console
git clone https://github.com/velasale/apple_proxy
```


Realsense camera install:

```console
sudo apt-get install ros-melodic-realsense2-camera
```

```console
sudo apt-get install ros-melodic-realsense2-description
```

## Only visualization (Rviz) 
Terminal 1 (Motion planner - MoveIt & visualization - Rviz):  
```console
roslaunch apple_proxy pickApp.launch
```

Terminal 2 (Our code):  
```console
rosrun apple_detection rob545_main.py
```

## With the real UR5 and visualize it in Rviz
Terminal 1 (UR5 drivers):  
```console
roslaunch apple_proxy ur5e_bringup.launch robot_ip:=169.254.177.232
```

Terminal 2 (Motion planner - MoveIt & visualization - Rviz):  
```console
roslaunch apple_proxy pickApp_real.launch
```

Terminal 3 (run command for camera):
```console
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true 
```

Terminal 4 (Our code):  
```console
rosrun apple_detection camera_test.py
```

Terminal 5 (Our code):  
```console
rosrun apple_detection rob545_main.py
```


