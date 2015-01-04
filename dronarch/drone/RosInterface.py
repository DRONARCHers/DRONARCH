__author__ = 'niclas'

from dronarch.sfm.Dronarch import Dronarch
from dronarch.helpers.debug import debug
from time import sleep
import rospy, subprocess
import talker
import os, signal
from ardrone_autonomy.srv import LedAnim
from std_msgs.msg import Empty
from ArdroneVideo import ArdroneVideo

class RosInterface:

    ros = None
    def __init__(self):
        self.processes = None
        self.ros_running = False
        #Can't initialize before roscore is running
        self.ardrone_video = None

    def start_ardrone(self):
        #start ardrone driver
        command = 'roslaunch tum_ardrone ardrone_driver.launch'
        p1 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

        sleep(10)
        command = 'rosrun tum_ardrone drone_stateestimation'
        p2 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

        sleep(10)
        command = 'rosrun tum_ardrone drone_autopilot'
        p3 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        return [p1, p2, p3]
        # p3.wait()

    def start_ros(self):
        try:
            self.processes = self.start_ardrone()
            rospy.init_node('dronarch')#, anonymous=True)
            self.ros_running = True
            self.ardrone_video = ArdroneVideo(img_out_dir=Dronarch.get_instance().vid_dest_dir)
        except Exception as e:
            debug(2, 'RosInterface exception: ', e.message)
        return self

    def stop_ros(self):
            debug(0, 'Ending ROS processes')
            for p in self.processes:
                os.killpg(p.pid, signal.SIGTERM)
            self.ros_running = False

    def publish(self, topic_name, msg_type, msg):
        talker.publish(topic_name=topic_name, msg_type=msg_type ,message=msg)

    def call_service(self, service_name, service_class, msg):
        talker.call_service(service_name=service_name, service_class=service_class, msg=msg)

    def start_video(self, FPS):
        self.ardrone_video.start_video(FPS=FPS)

    def stop_video(self):
        self.ardrone_video.stop_video()

    def anim_led(self):
        self.call_service(service_name='/ardrone/setledanimation', service_class=LedAnim, msg=(4, 1, 10))

    def land(self):
        self.publish(topic_name='/ardrone/land', msg_type=Empty, msg=None)

    def takeoff(self):
        self.publish(topic_name='/ardrone/takeoff', msg_type=Empty, msg=None)

    @classmethod
    def get_instance(self):
        if RosInterface.ros == None:
            RosInterface.ros = RosInterface()
        return RosInterface.ros

if __name__=='__main__':
    try:
        ros = RosInterface.get_instance()
        ros.start_ros()
        sleep(10)
        ros.anim_led()
        # ros.takeoff()
        sleep(1000)
        ros.land()
        # sleep(30)
    finally:
        ros.stop_ros()




