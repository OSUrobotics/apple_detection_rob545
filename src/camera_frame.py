#!/usr/bin/python


import rospy

import tf
import os
import numpy as np
from math import pi
import csv
import glob
import sys




if __name__ == '__main__':

    ###################################################################################
    # Creates the camera, world, and the aruco_end_effector frames to TF to be used later 
    ###################################################################################

    rospy.init_node('camera_frame_tf')

    # camera_transform = np.array([
    #                             [1, 0, 0, 0.02],
    #                             [0, 1, 0, -0.1],
    #                             [0, 0, 1, 0],
    #                             [0, 0, 0, 1]])

    # setting up the camera frame
    # trans2 = tf.transformations.translation_from_matrix(camera_transform)
    # rot2 = tf.transformations.quaternion_from_matrix(camera_transform)
    camera_tranlation = (0.0221, -0.0538, 0.05223)
    camera_rot = tf.transformations.quaternion_from_euler(0, 0, pi/4)
    
    
    camera_world = tf.TransformBroadcaster()
    

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        
        camera_world.sendTransform(camera_tranlation, tuple(camera_rot), rospy.Time.now(), 'camera_frame', 'tool0')
        
        rate.sleep()