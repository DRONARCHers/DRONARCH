__author__ = 'niclas'

import rospy, subprocess
from std_msgs.msg import String
from dronarch.helpers import helpers
from thread import start_new_thread
from time import sleep



def start_ros():
    #start roscore
    command = 'roscore'
    p1 =  subprocess.Popen(command, shell=True)

def start_ardrone():
    #start ardrone driver
    command = 'roslaunch tum_ardrone ardrone_driver.launch'
    p2 =  subprocess.Popen(command, shell=True)
    #
    # if success ==1:
    #     exit(success)
    #
    sleep(10)
    command = 'roslaunch tum_ardrone tum_ardrone.launch'
    p3 =  subprocess.Popen(command, shell=True)
    p3.wait()

def listen():

    rospy.Subscriber('chatter',String , callback)

def callback(args):
    print(args)

def talk():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = 'hello aliens'
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__=='__main__':
    start_new_thread(start_ardrone,())
    rospy.init_node('dronarch', anonymous=True)
    sleep()