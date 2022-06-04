#! /usr/bin/env python


import rospy

import tf
import os
import numpy as np
from math import sin, cos, pi
import csv
import glob
import sys
from pathlib import Path

from cv_bridge import CvBridge, CvBridgeError
import cv2
import csv
from sensor_msgs.msg import PointCloud2, Image, JointState
import sensor_msgs.point_cloud2 as pc2
from apple_detection.srv import CollectImageData, CollectImageDataResponse
import pickle



class ImageCollector:

	def __init__(self):
		self.point_cloud_data = None
		self.rgb_image = None
		self.depth_image = None
		self.depth_array = None
		self.joint_angles = []
		self.bridge = CvBridge()
		self.pose_number = 0
		self.working_directory = os.getcwd()


		
	def save_data(self, request):
		if request.trigger == True:
			directory_loc = self.working_directory + "/data/pose{}".format(self.pose_number)
			self.make_directory(directory=directory_loc)
			cv2.imwrite("{}/rgb_pose{}.bmp".format(directory_loc,self.pose_number), self.rgb_image)
			cv2.imwrite("{}/depth_pose{}.bmp".format(directory_loc,self.pose_number), self.depth_image)
			np.savetxt("{}/depth_pose{}.csv".format(directory_loc,self.pose_number), self.depth_array, delimiter=",")

			with open("{}/PointCloud_pose{}.pickle".format(directory_loc, self.pose_number), mode="wb") as pickle_file:
				pickle.dump(self.point_cloud_data, pickle_file, protocol=2)

			# np.savetxt("{}/PointCloud_pose{}.csv".format(directory_loc, self.pose_number), self.point_cloud_data, delimiter=",")
			transform = self.get_transform()

			file = open("{}/transform_pose{}.csv".format(directory_loc, self.pose_number), "w")
			wr = csv.writer(file, dialect='excel')
			for i in range(len(transform)):
				wr.writerow(transform[i])
			file.close()
			# np.savetxt("{}/transform_pose{}.csv".format(self.pose_number), transform, delimiter=",")
			# np.savetxt("{}/joint_angles_pose{}.csv".format(self.pose_number), self.joint_angles, delimiter=",")
			file = open("{}/joint_angles_pose{}.csv".format(directory_loc, self.pose_number), "w")
			wr = csv.writer(file, dialect='excel')
			for i in range(len(transform)):
				wr.writerow(transform[i])
			file.close()
			self.pose_number += 1
			
			return CollectImageDataResponse(1)
		else:
			return CollectImageDataResponse(0)

	def make_directory(self, directory):
		if not os.path.isdir(directory):
			Path(directory).mkdir(parents=True)
	
	def collect_point_cloud(self, point_cloud):
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
		self.point_cloud_data = np.array(point_cloud_list)
		# print(len(point_cloud_list))
		


	def collect_rgb(self, image):
		# print(image.encoding)
		# bridge = CvBridge()
		self.rgb_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		# print(cv2_img)
		# cv2.imwrite('test'+'.bmp', cv2_img)
		# rospy.signal_shutdown("done")

	def collect_depth(self, image):
		# print(type(image.data))
		self.depth_image = self.bridge.imgmsg_to_cv2(image, "passthrough")
		self.depth_array = np.array(self.depth_image, dtype=np.float)
		# print(im)
		# np.savetxt("depth.csv", im, delimiter=",")
		# cv2.imwrite("depth_test.bmp", cv2_img)


	def collect_depth_color(self, image):
		# print(image.encoding)

		bridge = CvBridge()
		cv2_img = bridge.imgmsg_to_cv2(image, "passthrough")
		cv2.imwrite("depth_color_test.bmp", cv2_img)

	def collect_joint_angles(self, angles):
		self.joint_angles =  angles.position

	def get_transform(self):
		listener = tf.TransformListener()
		
		while True:
			try:
				# translation, rotation = listener.lookupTransform('base_link', 'tool0', rospy.Time()) # change to the ur5
				translation, rotation = listener.lookupTransform('base_link', 'camera_frame', rospy.Time()) # change to the ur5
				break  # once the transform is obtained move on
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue  # if it fails try again

		transform_mat = listener.fromTranslationRotation(translation, rotation)

		return transform_mat



if __name__ == "__main__":

	rospy.init_node('camera_node', argv=sys.argv)

	image = ImageCollector()

	service = rospy.Service('collect_image', CollectImageData, image.save_data)
	
	sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, image.collect_point_cloud)
	sub2 = rospy.Subscriber('/camera/color/image_raw', Image, image.collect_rgb)
	# sub2 = rospy.Subscriber('/camera/depth/image_rect_raw', Image, image.collect_depth)
	sub3 = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, image.collect_depth)
	
	subscriber = rospy.Subscriber('/joint_states', JointState, image.collect_joint_angles)
	# sub2 = rospy.Subscriber('/camera/extrinsics/depth_to_color', Image, image.collect_depth_color)

	rospy.spin()
	