import os.path
from threading import Lock
from multiprocessing import Process
from thread import start_new_thread

#ros and opencv imports
import rospy, rostopic, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#dronarch imports
from dronarch.helpers.debug import debug
from dronarch.helpers import helpers


class ArdroneVideo:
    """
    Captures images from the drone and stores it to img_out_dir

    """
    def __init__(self, img_out_dir, show_image=False, save_image=True):
        """

        :param img_out_dir: directory to save image
        :param show_image: Display each captured image in a window. This might reduce the framerate
        :param save_image: Save images to img_out_dir
        :return:
        """
        if not helpers.ros_core_is_running():
            debug(1, 'Can\'t start ArdroneVideo, roscore not running')
            return
        if not os.path.isdir(img_out_dir): #specified dir path is invalid
            debug(1, 'Invalid directory for output images from drone: ', img_out_dir, '. Using ./ instead')
            self.img_out_dir = './'
        else:
            self.img_out_dir = img_out_dir
        self.show_image = show_image
        self.save_image = save_image


        # Holds the image frame received from the drone and later processed by the GUI
        self.image = None
        self.imageLock = Lock()

        #Indicates if recording is running
        self.running = False
        #Recording process. Needed to be killed in the end
        self.process = None
        #Counter for filename numbering
        self.img_nr = 0

    def schedule_redraw(self, FPS):
        """
        CAUTION: Run this as a own thread, as it goes into a endless loop. And kill it in the end.
        Subscribes to ROS to get new images from the drone as soon as they are ready and
        starts an endless loop of triggering the redraw_callback() method in order to periodically save the last received image.
        :param FPS:
        :return:
        """
        # Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
        self.sub_video = rospy.Subscriber(name='/ardrone/image_raw', data_class=Image, callback=self.receive_image)
        self.running = True
        time = 1/float(FPS)
        debug(0, 'Starting ArdroneVideo, extracting images')
        while not rospy.is_shutdown() and self.running:
            self.redraw_callback()
            sleep(time)

    def start_video(self, FPS):
        """
        Starts capturing images from the drones camera and saving them to self.vid_dest_dir.
        Stop recording using stop_video()
        :param FPS: Number of images to save per second.
        Any number higher than 25 might lead to images being stored more than once, since the camera captures not more than 25 images per second
        The resulting framerate might vary and not be nicely equally distributed
        """
        if self.ros_core_is_running():
            self.process = start_new_thread(self.schedule_redraw, (FPS,))
        else:
            debug(1, 'Can\'t start image extraction from drone, roscore not running')
        return self


    def stop_video(self):
        """
        Stops recording images
        :return:
        """
        self.running = False
        try:
            self.process.kill()
        except Exception as e:
            debug(1, 'Error when stopping recording video: ', e.message)

    def redraw_callback(self):
        """
        Callback method, that is called by ros. It saves and/or shows the current image
        :return:
        """
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
        """
        Callback method for ROS. ROS will call it with the new image as data. The image is then stored as self.image
        :param data: New image from the drone
        :return:
        """
        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            self.image = data # Save the ros image for processing by the display thread
        finally:
            self.imageLock.release()



if __name__=='__main__':
    video =  ArdroneVideo(FPS=3, img_out_dir='../../roaming/vid_imgs/', show_image=True)
    try:
        video.start_video()
        sleep(10)
    finally:
        video.stop_video()
