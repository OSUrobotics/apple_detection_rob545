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

    camera_transform = np.array([
                                [1, 0, 0, 0.02],
                                [0, 1, 0, -0.1],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

    # setting up the camera frame
    trans2 = tf.transformations.translation_from_matrix(camera_transform)
    rot2 = tf.transformations.quaternion_from_matrix(camera_transform)
    
    # br = tf.TransformBroadcaster()
    camera_world = tf.TransformBroadcaster()
    # aruco_ee = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # br.sendTransform(tuple(trans), tuple(rot), rospy.Time.now(), 'world_frame', 'j2s7s300_link_base') # j2s7s300_end_effector
        camera_world.sendTransform(tuple(trans2), tuple(rot2), rospy.Time.now(), 'camera_frame', 'tool0')
        # aruco_ee.sendTransform(trans3, tuple(rot3), rospy.Time.now(), 'aruco_endeffector', 'j2s7s300_end_effector')
        rate.sleep()