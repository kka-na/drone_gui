from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal, QObject

import rospy
import rospkg
import roslib
import tf
import math

from geometry_msg.msg import Point
from std_msgs.msg import String, Int32
from detectnet_ros_msgs.msg import BBoxes, BBox

class DetectMode() :
	def __init__(self) :
		rospy.init_node('visualizer', anonymous=True)
		rospy.spin()
		self.set_topic()
		self.ms2_area_max = 5000 #pixel

	def sub_topic(self) :
		rospy.Subscriber("/mode_num",Int32, self.mode_callback)
		self.pub_miss1 = rospy.Publisher("/mission_mode/mission1", Point, queue_size = 1)
		self.pub_miss2 = rospy.Publisher("/mission_mode/mission2", Int32, queue_size = 1)

	def d_bboxes_callback(self, data) :
		if self.mode_num == 1 :
			for bbox in data.bboxes :
				if bbox.id == 1 :
					self.ms1_cnt = self.ms1_cnt + 1
					if self.ms1_cnt == 0 :
						self.ms1_cx_old = bbox.cx
					elif self.ms1_cnt % 50 == 0 :
						minus = abs(self.ms1_cx_old - bbox.cx)
						# 1280 /6 = 214 (180/6 = 30*)
						if minus <= 214 :
							angle = 30
						elif minus > 214 && minus <= 428 :
							angle = 60
						elif minus > 428 && minus <= 642 :
							angle = 90
						if self.ms1_cx_old > bbox.cx :
							minus = minus * -1 #old cx - new cx
							angle = angle * -1 #convert to degree
							
						point = Point()
						point.x = minus
						point.y = angle
						self.pub_miss1.publish(point)
						self.ms1_cx_old = bbox.cx

		elif self.mode_num == 2 :
			for bbox in data.bboxes :
				if self.ms2_mode == True && bbox.id == 13 :
					if bbox.area >= self.ms2_area_max :
						self.ms2_cnt = self.ms2_cnt + 1
						if self.ms2_cnt == 20 :
							ms2_int = Int32()
							ms2_int.data = 1
							self.pub_miss2.publish(ms2_int)
							self.ms2_mode = False
					

	def mode_callback(self,data) :
		self.ms1_cx_old = 0
		self.ms1_cnt = 0
		self.ms2_mode = True
		self.ms2_cnt = 0

		self.mode_num = data.data
		rospy.Subscriber("/nano_detection/bboxes", BBoxes, self.d_bboxes_callback) 


		
#test
dmode = DetectMode()
