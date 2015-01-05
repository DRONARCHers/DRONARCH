__author__ = 'niclas'


import rospy, subprocess, psutil, os, signal

#ros imports
from ardrone_autonomy.srv import LedAnim
from ArdroneVideo import ArdroneVideo
from std_msgs.msg import Empty

#dronarch imports
from dronarch.sfm.Dronarch import Dronarch
from dronarch.helpers.debug import debug
from talker import DronarchTopics, publish, call_service

class RosInterface:
    """
    CAUTION: Implements the Singleton pattern, so get_instance() MUST be used
    Serves as interface to ALL ROS interaction.
    Any further ROS interaction has to be added to this class.

    """
    ros = None
    def __init__(self):
        if not self.ros == None:
            debug(2, 'Cannot create a second instance of an class implemening the singleton pattern. Use get_instance() instead.')
            return
        self.processes = None
        self.ros_running = False
        #Can't initialize before roscore is running
        self.ardrone_video = None

    def start_ardrone(self):
        """
        Starts the ros processes related to the ardrone_autonomy and tum_ardrone ros nodes
        :return:
        """
        #start ardrone driver
        command = 'roslaunch tum_ardrone ardrone_driver.launch'
        p1 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

        rospy.sleep(10)
        command = 'rosrun tum_ardrone drone_stateestimation'
        p2 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)


        command = 'rosrun tum_ardrone drone_autopilot'
        p3 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        rospy.sleep(10)
        return [p1, p2, p3]

    def start_ros(self):
        """
        Start all ROS related processes. If ROS is already running, it will be stopped and restarted.
        If any ardrone_autonomy and tum_ardrone related ros nodes are still running, these will be killed.
        :return:
        """
        if self.ros_running:
            debug(1, 'ROS already running. Will stop and restart ROS')
            self.stop_ros()
        try:
            #Make sure there are no other ardrone, tum_ardrone and so on ROS-processes running
            ros.kill_ros()

            self.processes = self.start_ardrone()
            rospy.init_node('dronarch')#, anonymous=True)
            self.ros_running = True
            #This might initialize the dronarch.sfm.Dronarch class, as it accesses it's single instance. But this is not a problem.
            self.ardrone_video = ArdroneVideo(img_out_dir=Dronarch.get_instance().vid_dest_dir)
        except Exception as e:
            debug(2, 'RosInterface exception: ', e.message)

    def stop_ros(self):
        """
        Stops all ros processes started by this instance. To kill all ros processes related to the
        ardrone_autonomy and tum_ardrone nodes, use self.kill_ros()
        :return:
        """
        debug(0, 'Ending ROS processes')
        for p in self.processes:
            os.killpg(p.pid, signal.SIGTERM)
        self.ros_running = False
    def kill_ros(self):
        """
        Kills all ardrone_autonomy and tum_ardrone related processes, no matter how they have been started
        TODO: Not sure if this is working on other OS than the tested Ubuntu as well.
        :return:
        """
        #TODO: Not sure if this is working on other OS than the tested Ubuntu as well.
        for proc in psutil.process_iter():
            # print proc.name
            if 'ardrone_driver' in proc.name or 'drone_autopilot' in proc.name or 'drone_stateestimation' in proc.name or 'tum_ardrone' in proc.name or 'drone_gui' in proc.name:
                debug(1, 'ROS-process already running. Killing ', proc.name)
                proc.kill()

    def start_video(self, FPS):
        """
        Starts capturing images from the drones camera and saving them to Dronarch.vid_dest_dir.
        Stop recording using stop_video()
        :param FPS: Number of images to save per second.
        Any number higher than 25 might lead to images being stored more than once, since the camera captures not more than 25 images per second
        The resulting framerate might vary and not be nicely equally distributed
        :return:
        """
        self.ardrone_video.start_video(FPS=FPS)

    def stop_video(self):
        """
        Stops saving images from the drone.
        To be called after start_video()
        :return:
        """
        self.ardrone_video.stop_video()

    def anim_led(self):
        """
        Triggers an animation of the drones LEDs
        :return:
        """
        call_service(service_name='/ardrone/setledanimation', service_class=LedAnim, msg=(4, 1, 10))

    def land(self):
        """
        Lands the drone
        :return:
        """
        publish(publisher=DronarchTopics.pub_land, message=Empty())

    def takeoff(self):
        """
        Tells the drone to takeoff.
        :return:
        """
        publish(publisher=DronarchTopics.pub_takeoff, message=Empty())

    def reset(self):
        """
        Resets the drone.
        :return:
        """
        publish(publisher=DronarchTopics.pub_reset, message=Empty())

    @classmethod
    def get_instance(self):
        """
        Get the single instance of RosInterface. Use this method instead of the constructor.
        :return:
        """
        if RosInterface.ros == None:
            RosInterface.ros = RosInterface()
        return RosInterface.ros

if __name__=='__main__':
    try:
        ros = RosInterface.get_instance()
        ros.start_ros()
        # rospy.sleep(10)
        ros.anim_led()
        # ros.takeoff()
        # rospy.sleep(1)
        ros.reset()
        rospy.sleep(1)
        ros.land()
        rospy.sleep(1)
    finally:
        ros.land()
        rospy.sleep(1)
        ros.stop_ros()




