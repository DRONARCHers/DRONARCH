#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies

import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock
import sched
import time

from cv_bridge import CvBridge
import cv2
from threading import Timer
import numpy as np
from subprocess import Popen

# Some Constants
CONNECTION_CHECK_PERIOD = 40
GUI_UPDATE_PERIOD = 20

class DroneVideoDisplay:

	def __init__(self):

		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		self.schedule_redraw(GUI_UPDATE_PERIOD)
		# self.RedrawCallback()
		# time.sleep(5)
		# self.RedrawCallback()

	def schedule_redraw(self, time):
		while True:
			self.RedrawCallback()
			key = cv2.waitKey(time)
			if not key == -1:
				break


	def ConnectionCallback(self):
		print('callback')
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the ROS image into a QImage which we can display
					# image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
					bridge = CvBridge()
					image = bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
			finally:
				self.imageLock.release()

			cv2.imshow(winname='Video',mat=image)

			cv2.imwrite('img.jpg', image)


	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
		finally:
			self.imageLock.release()


if __name__=='__main__':
	# cv2.imshow('bl', np.ones([512,512,3]))

	rospy.init_node('ardrone_video_display')
	display = DroneVideoDisplay()
