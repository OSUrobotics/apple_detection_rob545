# apple_detection_rob545
Repo for rob545 final project of finding the center of an apple relative to the arm base.


This looks to be the RealSense repo needed for the camera:
https://github.com/IntelRealSense/realsense-ros

## How to run it

### Within Rviz 
Terminal 1:
`roslaunch apple_proxy pickApp.launch`

Terminal 2:
`rosrun apple_detection rob545_main.py`

### With the real UR5 and visualize it in Rviz


