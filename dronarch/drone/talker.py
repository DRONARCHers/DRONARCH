__author__ = 'niclas'
import rospy
from dronarch.helpers.debug import debug
from dronarch.helpers import helpers
from ardrone_autonomy.srv import LedAnim
from std_msgs.msg import Empty

def publish(topic_name, msg_type, message, queue_size=10, repeat=False, sending_rate=1):
    pub = rospy.Publisher(name=topic_name, data_class=msg_type, queue_size=queue_size)
    try:
        if repeat:
            debug(0, 'Sending {} to ros topic {} with a frequency of {}Hz'.format(message, topic_name, sending_rate))
            rate = rospy.Rate(sending_rate)
            while not rospy.is_shutdown():
                rospy.loginfo(message)
                __publish__(pub=pub, message=message, msg_type=msg_type)
                rate.sleep()
        else:
            if helpers.ros_core_is_running():
                debug(0, 'Sending {} to ros topic {} once'.format(message, topic_name))
                __publish__(pub=pub, message=message, msg_type=msg_type)
            else:
                debug(1, 'Can not publish to ros. Roscore not running')
    except TypeError as e:
        debug(1, 'Can\'t publish to {}: {}'.format(topic_name, e.message))

def __publish__(pub, message, msg_type):
    if msg_type == Empty:
        pub.publish(Empty())
        rospy.loginfo('Empty published')
    else:
        pub.publish(message)
        rospy.loginfo(message)

def call_service(service_name, service_class, msg):
    rospy.wait_for_service(service_name)
    try:
        debug(0, 'Calling ROS service ', service_name)
        service = rospy.ServiceProxy(name=service_name, service_class=service_class)
        if service_class == LedAnim:
            response = service(msg[0], msg[1], msg[2])
        return response
    except rospy.ServiceException as e:
        debug(1, 'ROS service call failed: ', e)