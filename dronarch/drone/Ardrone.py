__author__ = 'niclas'

from dronarch.sfm import Dronarch
from dronarch.helpers.debug import debug
from thread import start_new_thread
from time import sleep
import rospy, subprocess
from multiprocessing import Process
import talker
import os, signal
from ardrone_autonomy.srv import LedAnim

class Ardrone:
    def __init__(self):
        self.processes = None
        self.ros_running = False

    def start_ardrone(self):
        #start ardrone driver
        command = 'roslaunch tum_ardrone ardrone_driver.launch'
        p1 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

        sleep(10)
        command = 'rosrun tum_ardrone drone_stateestimation'
        p2 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

        command = 'rosrun tum_ardrone drone_autopilot'
        p3 = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        return [p1, p2, p3]
        # p3.wait()

    def __enter__(self):
        try:
            self.processes = self.start_ardrone()
            rospy.init_node('dronarch', anonymous=True)
            self.ros_running = True
        except Exception as e:
            debug(2, 'Ardrone exception: ', e.message)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
            debug(0, 'Ending ROS processes')
            for p in self.processes:
                os.killpg(p.pid, signal.SIGTERM)
            self.ros_running = False
    def call_service(self, service_name, service_class, msg):
        talker.call_service(service_name=service_name, service_class=service_class, msg=msg)

    def capture_imgs(self):
        pass
if __name__=='__main__':
    with Ardrone() as ardrone:
        ardrone.call_service(service_name='/ardrone/setledanimation', service_class=LedAnim, msg=(4, 1, 10))
        sleep(10000)




