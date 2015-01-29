__author__ = 'niclas'

import rospy, subprocess
from std_msgs.msg import String,Empty
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

    sleep(10)
    command = 'roslaunch tum_ardrone tum_ardrone.launch'
    p3 =  subprocess.Popen(command, shell=True)
    p3.wait()

def listen():

    rospy.Subscriber('chatter',String , callback)

def callback(args):
    print 'blub '+str(args)
    pub.publish(Empty())

def talk():
    pub = rospy.Publisher('/ardrone/land', Empty, queue_size=100, latch=True)
    # rospy.sleep(10)
    # rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     msg = 'hello aliens'
        # rospy.loginfo()
    pub.publish(Empty())
    # rospy.Timer(period=rospy.Duration(0.5), callback=callback, oneshot=False)
    # rospy.spin()
    # pub.publish(Empty())
    rospy.sleep(10)

if __name__=='__main__':
    # start_new_thread(start_ardrone,())
    rospy.init_node('TESTER', anonymous=True)
    # pub = rospy.Publisher('/ardrone/land', Empty, queue_size=100, latch=True)
    talk()
    # sleep(1000)