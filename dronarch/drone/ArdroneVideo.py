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

from cv_bridge import CvBridge
import cv2
from threading import Lock
from multiprocessing import Process
from time import sleep
from dronarch.helpers.debug import debug
from dronarch.helpers import helpers
import rostopic
import os.path

from thread import start_new_thread

class ArdroneVideo:
    def __init__(self, img_out_dir, show_image=False, save_image=True):
        if not helpers.ros_core_is_running():
            debug(1, 'Can\'t start ArdroneVideo, roscore not running')
        else:
            pass
            # rospy.init_node('dronarch_video', anonymous=True)
        if not os.path.isdir(img_out_dir):
            debug(1, 'Invalid directory for output images from drone: ', img_out_dir, '. Using ./ instead')
            self.img_out_dir = './'
        else:
            self.img_out_dir = img_out_dir
        self.show_image = show_image
        self.save_image = save_image


        # Holds the image frame received from the drone and later processed by the GUI
        self.image = None
        self.imageLock = Lock()

        self.running = False
        self.process = None
        self.img_nr = 0


    def start_video(self, FPS):
        if self.ros_core_is_running():
            self.process = start_new_thread(self.schedule_redraw, (FPS,))
        else:
            debug(1, 'Can\'t start image extraction from drone, roscore not running')
        return self


    def schedule_redraw(self, FPS):
        # Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
        self.sub_video = rospy.Subscriber(name='/ardrone/image_raw', data_class=Image, callback=self.receive_image)
        self.running = True
        time = 1/float(FPS)
        debug(0, 'Starting ArdroneVideo, extracting images')
        while not rospy.is_shutdown() and self.running:
            self.redraw_callback()
            sleep(time)

    def stop_video(self):
        self.running = False
        try:
            self.process.kill()
        except Exception:
            pass

    def redraw_callback(self):
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
                if self.show_image:
                    cv2.imshow(winname='Video',mat=image)
                    cv2.waitKey(1)
                if self.save_image:
                    file_name=self.img_out_dir+'ardrone_'+str(self.img_nr)+'.jpg'
                    cv2.imwrite(file_name, image)
                    self.img_nr = self.img_nr+1
                    # debug(0, 'Image from ardrone has been saved to ', file_name)



    def receive_image(self,data):
        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            self.image = data # Save the ros image for processing by the display thread
        finally:
            self.imageLock.release()



if __name__=='__main__':
    video =  ArdroneVideo(FPS=3, img_out_dir='../../roaming/vid_imgs/', show_image=True)
    video.start_video()
    sleep(10)
    video.stop_video()
