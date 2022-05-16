# apple_detection_rob545
Repo for rob545 final project of finding the center of an apple relative to the arm base.


This looks to be the RealSense repo needed for the camera:
https://github.com/IntelRealSense/realsense-ros

# Instructions

## 1 - Install additional required packages
Step in the same workspace: ~/YOUR_WS/src/  
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


## 2 - Make sure that you source your ws
Option 1 - Everytime you open a terminal
```console
source ~/YOUR_WS/devel/setup.bash
```

Option 2 - Edit the ".bashrc" file, so you do not need to run it everytime
```console
gedit ~/.bashrc
```

and add the text from Option 1 at the end of the file t


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
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true ordered_pc:=true allow_no_texture_points:=true
```

Terminal 4 (Our code):  
```console
rosrun apple_detection camera_test.py
```

Terminal 5 (Our code):  
```console
rosrun apple_detection rob545_main.py
```


