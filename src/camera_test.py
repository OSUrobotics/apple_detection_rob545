#! /usr/bin/env python


import rospy

import tf
import os
import numpy as np
from math import sin, cos, pi
import csv
import glob
import sys

from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2

def get_transform():
    listener = tf.TransformListener()
    
    while True:
        try:
            translation, rotation = listener.lookupTransform('j2s7s300_link_base', 'j2s7s300_end_effector', rospy.Time()) # change to the ur5
            break  # once the transform is obtained move on
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue  # if it fails try again

    transform_mat = listener.fromTranslationRotation(translation, rotation)


    file = open(dir_path + "/data/TransformMatrix_" + str(n) + ".csv", "w")
    wr = csv.writer(file, dialect='excel')

    for i in range(len(transform_mat)):
        wr.writerow(transform_mat[i])
    
    file.close()

def save_point_cloud(point_cloud):
    # print(point_cloud.row_step / point_cloud.point_step * point_cloud.height)
    # print(point_cloud.height, point_cloud.width, point_cloud.width * point_cloud.height)
    # print(point_cloud.point_step)
    # points = pc2.read_points(point_cloud)
    # print(points[:])
    point_cloud_list = []
    for p in pc2.read_points(point_cloud):
        point_cloud_list.append(p[:])
        # print(" x : %f  y: %f  z: %f r: %f" %(p[0],p[1],p[2], p[3]))
    
    # k = 0
    # new_new_list = []
    # for  i in range(len(point_cloud_list)/point_cloud.width):
    #     new_list = []
    #     for j in range(point_cloud.width):
    #         new_list.append(point_cloud_list[k])
    #         k += 1
    #     new_new_list.append(new_list)

    # point_cloud_np = np.array(new_new_list, dtype=np.float)
    point_cloud_np = np.array(point_cloud_list)
    # print(len(point_cloud_list))
    np.savetxt("point_cloud2.csv", point_cloud_np, delimiter=",")
    rospy.signal_shutdown("done")


def save_rgb(image):
    # print(image.encoding)
    bridge = CvBridge()
    cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
    print(cv2_img)
    cv2.imwrite('test'+'.bmp', cv2_img)
    rospy.signal_shutdown("done")

def save_depth(image):
    print(type(image.data))

    bridge = CvBridge()
    cv2_img = bridge.imgmsg_to_cv2(image, "passthrough")
    im = np.array(cv2_img, dtype=np.float)
    # print(im)
    # np.savetxt("depth.csv", im, delimiter=",")
    # cv2.imwrite("depth_test.bmp", cv2_img)

    rospy.signal_shutdown("done")

def save_depth_color(image):
    print(image.encoding)

    bridge = CvBridge()
    cv2_img = bridge.imgmsg_to_cv2(image, "passthrough")
    cv2.imwrite("depth_color_test.bmp", cv2_img)

    rospy.signal_shutdown("done")


if __name__ == "__main__":

    rospy.init_node('camera_node', argv=sys.argv)

    sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, save_point_cloud)
    # sub2 = rospy.Subscriber('/camera/color/image_raw', Image, save_rgb)
    # sub2 = rospy.Subscriber('/camera/depth/image_rect_raw', Image, save_depth)
    # sub2 = rospy.Subscriber('/camera/extrinsics/depth_to_color', Image, save_depth_color)

    rospy.spin()
    