#! /usr/bin/env python
import sys
import copy
import numpy as np
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import math

import signal
import os
import tf

import rospy
import rospkg
import roslib

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5 import uic

from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Point
from sensor_msgs.msg import Image,CompressedImage, Imu, NavSatFix, BatteryState
from nav_msgs.msg import Odometry


class ControlRos(QObject) :
	def __init__(self) :
		super(ControlRos, self).__init__()
		rospy.init_node('visualizer', anonymous=True)
		#test
		rospy.Subscriber('/pose', Odometry, self.imuCallBack)

	#test
	send_imu = pyqtSignal(float, float, float)
	def imuCallBack(self, data) :
		#test
		roll = data.pose.pose.position.z
		pitch = -data.pose.pose.position.z
		yaw = data.pose.pose.position.z * 1.5
		quaternion = (data.pose.pose.position.z,-data.pose.pose.position.z,data.pose.pose.position.z,-data.pose.pose.position.z  )
		euler = tf.transformations.euler_from_quaternion(quaternion)	

		self.send_imu.emit(roll, pitch, yaw)

	send_drone_pos = pyqtSignal(str)
	def d_pos_callback(self, data) :
		pos_txt = "lat : {}, lng : {}".format(round(data.latitude,5), round(data.longitude,5))
		self.send_drone_pos.emit(pos_txt)

	send_drone_ypr = pyqtSignal(str)
	def d_imu_callback(self, data) :
		quaternion = (data.orientation.x,data.orientation.y, data.orientation.z,data.orientation.w  )
		euler = tf.transformations.euler_from_quaternion(quaternion)	
		PI = 3.141592
		pitch = round(euler[0]*180/PI, 4)
		roll = round(euler[1]*180/PI, 4)
		yaw = round(euler[2]*180/PI, 4)

		ypr_txt = "Y:{}   P:{}  R:{}".format(yaw, pitch, roll)
		self.send_drone_ypr.emit(ypr_txt)
		self.send_imu.emit(roll, pitch, yaw)

	send_drone_alt = pyqtSignal(str)
	def d_alt_callback(self, data) :
		alt_txt = "{} m".format(round(data.pose.position.z,3))
		self.send_drone_alt.emit(alt_txt)

	send_drone_batt = pyqtSignal(str) 
	def d_batt_callback(self, data) :
		bat_txt = "{} v".format(round(data.voltage,3))
		self.send_drone_batt.emit(bat_txt)

	send_drone_cam = pyqtSignal(object)
	def d_cam_callback(self, data) :
		np_arr = np.fromstring(data.data, np.uint8)
		cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		self.send_drone_cam.emit(rgbImage)


	def initialize_ros(self) : 
		rospy.Subscriber("/camera_nano/object_detect/image_raw/compressed", CompressedImage, self.d_cam_callback)
		rospy.Subscriber("/mavros/imu/data", Imu, self.d_imu_callback)
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.d_pos_callback)
		rospy.Subscriber("/mavros/battery", BatteryState , self.d_batt_callback)
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.d_alt_callback)


		




