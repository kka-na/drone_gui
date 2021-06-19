#!/usr/bin/env python
import rospy
import threading
import cv2 
import numpy as np
import sys, time, signal
from sensor_msgs.msg import CompressedImage


class DroneTV(threading.Thread) :
	def __init__(self) :
		threading.Thread.__init__(self)
		self.cap = cv2.VideoCapture(0) #now web cam
		self.thread_running = True
		rospy.init_node("visualizer",anonymous=True)
		self.pub_tello_img = rospy.Publisher('/tello_img/image_raw/compressed', CompressedImage,  queue_size = 1)

	def run(self) :
		msg  = CompressedImage()
		while self.thread_running :
			b, frame = self.cap.read()
			if b:
				msg.header.stamp = rospy.Time.now()
				msg.format =  "jpeg"
				encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),30]
				msg.data = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tostring()
				self.pub_tello_img.publish(msg)
			else : 
				pass

	def stop(self) :
		thread_running = False
		#self.quit()
		#self.wait(100)

try :
	drontv = DroneTV()
	print("Start ... ")
	drontv.start()
	signal.pause()
except KeyboardInterrupt :
	drontv.stop()
	print("Shut down .. ")