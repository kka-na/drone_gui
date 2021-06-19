#! /usr/bin/env python
import sys
import copy
import tf
import numpy as np
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import math
from time import sleep
from datetime import datetime
from pytz import timezone
import signal
import os
import subprocess, shlex, psutil
import csv 

import rospy
import rospkg
import roslib
import rosbag
import rviz
import actionlib

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5 import uic

from geometry_msgs.msg import Twist, PoseStamped,PointStamped, Point
from actionlib_msgs.msg import GoalID
from control_msgs.msg import PointHeadAction, PointHeadGoal

from sensor_msgs.msg import Image,CompressedImage, Imu, NavSatFix, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32


import controlros
import imugl

#call ui file
form_class = uic.loadUiType("dgui.ui")[0]

playbar_start = 0
playbar_temp = 0
playbar_end = 0
play_btn_clicked = False
pause_btn_clicked = False

save_as_dataset = False
img_path = ""
drone_cnt = 0

class WindowClass(QMainWindow, form_class) :
	def __init__(self) : 
		super(WindowClass, self).__init__()
		self.setupUi(self)
		self.rosbag_proc = subprocess
		self.droGL = imugl.ImuGL("Drone.obj")
		self.dro_gl.addWidget(self.droGL)
		self.droGL.show()
		self.ctrlros = controlros.ControlRos()
		self.loadImageFromFile() 
		self.connect_signals()

	def connect_signals(self) :
		#Monitoring
		self.actionReady.triggered.connect(self.actionready_triggered) #ready state
		self.actionRecord.triggered.connect(self.actionrecord_triggered)
		self.actionMission1.triggered.connect(self.actionmission1_triggered)
		self.actionMission2.triggered.connect(self.actionmission2_triggered) #start state
		self.actionStop.triggered.connect(self.actionstop_triggered) #stop state
		#Playback
		self.actionReady_2.triggered.connect(self.actionready_2_triggered)
		self.actionSave_as_Dataset.triggered.connect(self.actionSave_as_Dataset_triggered)
		self.actionStart_2.triggered.connect(self.actionstart_2_triggered) #start state

		self.ctrlros.send_drone_pos.connect(self.update_drone_pos)
		self.ctrlros.send_drone_ypr.connect(self.update_drone_ypr)
		self.ctrlros.send_drone_alt.connect(self.update_drone_alt)
		self.ctrlros.send_drone_batt.connect(self.update_drone_batt)
		self.ctrlros.send_drone_cam.connect(self.update_drone_cam)

	def actionready_triggered(self):
		print("Action Ready Triggered")
		self.initialize_all() #initialize rviz and ros nodes

	def actionrecord_triggered(self):
		print("Action Record Triggered")
		self.set_record()

	def actionmission1_triggered(self):
		print("Action Mission1 Triggered")
		mode = Int32()
		mode.data = 1
		self.pub_mode.publish(mode)

	def actionmission2_triggered(self):
		print("Action Mission2 Triggered")
		mode = Int32()
		mode.data = 2
		self.pub_mode.publish(mode)

	def actionstop_triggered(self) :
		print("Action Stop Triggered")
		self.monitoring_stop()

	def loadImageFromFile(self) :
		self.qPixmapDrone = QPixmap()
		self.qPixmapDrone.load("./icon/drone.png")
		self.qPixmapDrone = self.qPixmapDrone.scaledToWidth(50)
		self.drone_png.setPixmap(self.qPixmapDrone)

	def initialize_all(self) :
		self.ctrlros.initialize_ros()
		self.pub_mode = rospy.Publisher("/mode_num",Int32, queue_size = 1)
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Ready State")
		mbox.setText("All Process were Initialized     ")
		mbox.exec_()

	def set_record(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Record State")
		mbox.setText("All information will be recorded.     \nWould you like to record it?     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		returnv = mbox.exec_()
		path = ""
		if returnv == QMessageBox.Ok :
			path =  QFileDialog.getExistingDirectory(None, 'Select folder to save recorded information',QDir.currentPath(), QFileDialog.ShowDirsOnly)
			now_utc = datetime.now(timezone('UTC'))
			now_kst = now_utc.astimezone(timezone('Asia/Seoul'))
			dirname = "{}/drone_{}_dataset".format(str(path),str(now_kst.strftime('%Y%m%d%H%M%S')))
			os.mkdir(dirname)
			command ="rosbag record -o "+dirname+"/ -a"
			command = shlex.split(command)
			self.rosbag_proc = subprocess.Popen(command)


	@pyqtSlot(str)
	def update_drone_pos(self, string) :
		self.drone_pos_label.setText(string)

	@pyqtSlot(str)
	def update_drone_ypr(self, string) :
		self.drone_ypr_label.setText(string)

	@pyqtSlot(str)
	def update_drone_alt(self, string) :
		self.drone_alt_label.setText(string)

	@pyqtSlot(str)
	def update_drone_batt(self, string) :
		self.drone_batt_label.setText(string)

	@pyqtSlot(object)
	def update_drone_cam(self, rgbImage) :
		global img_path, save_as_dataset, drone_cnt
		if save_as_dataset:
			ts_path = "{}/drone/timestamp.txt".format(str(img_path))
			ts = int(str(data.header.stamp))
			ts /= 1000000000
			ts = str(datetime.fromtimestamp(ts, timezone('Asia/Seoul')))
			f = open(ts_path, 'a')
			f.write(ts+'\n')
			f.close()
			cv2.imwrite("{}/drone/{}.jpg".format(str(img_path),str(drone_cnt)), rgbImage)
			drone_cnt+=1
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(720,480, Qt.KeepAspectRatio)
		self.drone_cam_label.setPixmap(QPixmap.fromImage(qimage))
		self.drone_cam_label.show()
		QApplication.processEvents()



	def monitoring_stop(self) :
		for proc in psutil.process_iter() :
			if "record" in proc.name() : 
				proc.send_signal(subprocess.signal.SIGINT)
		self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
		rospy.on_shutdown(self.myhook) #shut down the nodes
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Stop State")
		mbox.setText("All Process were Ended     ")
		mbox.exec_()

	#shut down the ros nodes
	def myhook(self):
		print("Shut down all processes cleary!")

# ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ - End of Monitoring Processs - ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ #

	def actionready_2_triggered(self):
		print("Action Ready_2 Triggered")
		self.PlayBarThread = PlayBarThread()
		self.PlayBarThread.change_playbar_value.connect(self.change_playbar_state)
		self.setting_playback()
		self.initialize_all()

	def setting_playback(self) :
		self.bagfile =  QFileDialog.getOpenFileName(self, 'Select .bag files to playback', './')[0]
		self.bag = rosbag.Bag(self.bagfile)
		b_start = self.bag.get_start_time()
		b_end = self.bag.get_end_time()
		self.b_total_sec = int(b_end-b_start)
		self.slider.setRange(0, self.b_total_sec)
		self.slider.valueChanged.connect(self.setBagPosition)
		self.play_btn.clicked.connect(self.push_play_btn)
		self.pause_btn.clicked.connect(self.push_pause_btn)

	def actionSave_as_Dataset_triggered(self) :
		global save_as_dataset, img_path, drone_cnt
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Dataset Saving State")
		mbox.setText("Saving .bag data as Dataset.     \nWould you like to save it?     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		returnv = mbox.exec_()
		path = ""
		if returnv == QMessageBox.Ok :
			path =  QFileDialog.getExistingDirectory(None, 'Select folder to save Dataset',QDir.currentPath(), QFileDialog.ShowDirsOnly)
			os.mkdir("{}/images".format(str(path)))
			drone_cnt = 0 
			save_as_dataset = True
			

	def actionstart_2_triggered(self) :
		global playbar_start, playbar_end
		playbar_start = 0
		playbar_end = self.b_total_sec
		self.get_topic_list()
		command ="rosbag play "+self.bagfile
		command = shlex.split(command)
		self.rosbag_proc = subprocess.Popen(command)
		self.PlayBarThread.start()


	def setBagPosition(self) :
		global playbar_start
		playbar_start = self.slider.value()
		self.ts_txt.setText(str(self.slider.value()))

	def push_play_btn(self) :
		global playbar_start
		global play_btn_clicked
		print("play btn pushed")
		play_btn_clicked = True
		command ="rosbag play -s"+str(playbar_start)+" "+self.bagfile
		command = shlex.split(command)
		self.rosbag_proc = subprocess.Popen(command)
		self.PlayBarThread.start()

	def push_pause_btn(self) :
		global pause_btn_clicked
		print("pause btn pushed")
		pause_btn_clicked = True
		for proc in psutil.process_iter() :
			if "play" in proc.name() :
				proc.send_signal(subprocess.signal.SIGINT)
		self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
		self.PlayBarThread.stop()


	@pyqtSlot(int)
	def change_playbar_state(self, value) :
		self.slider.setValue(value)
		self.ts_txt.setText(str(value))

class PlayBarThread(QThread) :
	change_playbar_value = pyqtSignal(int)
	def __init__(self) :
		super(PlayBarThread, self).__init__()
		self.running = True

	def run(self) :
		global playbar_start, playbar_end, playbar_temp
		global play_btn_clicked, pause_btn_clicked
		if pause_btn_clicked :
			b_cnt = playbar_temp
			pause_btn_clicked = False
		else :
			b_cnt = playbar_start
		if play_btn_clicked :
			self.running = True

		while b_cnt in range(playbar_start, playbar_end+1) :
			if self.running == False : break
			if(b_cnt > playbar_end) :
				self.stop()
			self.change_playbar_value.emit(b_cnt)
			QApplication.processEvents()
			sleep(1)
			playbar_temp = b_cnt
			b_cnt += 1

	def stop(self) :
		self.running = False
		self.quit()
		self.wait(5000)


if __name__ == "__main__" :
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()


