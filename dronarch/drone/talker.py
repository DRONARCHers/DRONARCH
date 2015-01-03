__author__ = 'niclas'
import rospy
from dronarch.helpers.debug import debug
from ardrone_autonomy.srv import LedAnim

def talk(msg_type, message, queue_size=10, repeat=False, sending_rate=1):
    pub = rospy.Publisher(name='dronarch', data_class=msg_type, queue_size=queue_size)
    if repeat:
        debug(0, 'Sending {} to ros topic with a frequency of {}Hz'.format(message, sending_rate))
        rate = rospy.Rate(sending_rate)
        while not rospy.is_shutdown():
            rospy.loginfo(message)
            pub.publish(message)
            rate.sleep()
    else:
        if rospy.is_shutdown():
            debug(0, 'Sending {} to ros topic once'.format(message))
            rospy.loginfo(message)
            pub.publish(message)
        else:
            debug(1, 'Can not send to ros. Roscore not running?')

def call_service(service_name, service_class, msg):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(name=service_name, service_class=service_class)
        if service_class == LedAnim:
            response = service(msg[0], msg[1], msg[2])
    except rospy.ServiceException as e:
        debug(1, 'ROS service call failed: ', e)