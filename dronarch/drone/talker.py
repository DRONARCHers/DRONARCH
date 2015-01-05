__author__ = 'niclas'
#ros imports
import rospy
from ardrone_autonomy.srv import LedAnim
from std_msgs.msg import Empty,String

#dronarch imports
from dronarch.helpers.debug import debug
from dronarch.helpers import helpers

class DronarchTopics:
    """
    Enumerates all ROS topics needed.
    Add new topics, publisher, receiver here.
    """
    queue_size = 10
    latch = True

    pub_land = rospy.Publisher(name='/ardrone/land', data_class=Empty, queue_size=queue_size, latch=latch)
    pub_takeoff = rospy.Publisher(name='/ardrone/takeoff', data_class=Empty, queue_size=queue_size, latch=latch)
    pub_tum_ardrone = rospy.Publisher(name='/tum_ardrone/com', data_class=String, queue_size=queue_size, latch=latch)
    pub_reset = rospy.Publisher(name='/ardrone/reset', data_class=String, queue_size=queue_size, latch=latch)


#
#   The following are a set of functions used to communicate with ROS

def publish(publisher, message):
    """
    Publishes the message to a specified publisher.
    :param publisher: Has to be one declared in the DronarchTopics class
    :param message: Message to send. It's type has to correspond with the one specified in publisher
    # NOT USED :param repeat: Should it be send repeating in a endless loop
    # NOT USED :param sending_rate: Hz of repetition if repeat==True
    :return:
    """
    try:
        # if repeat:
        #     debug(0, 'Sending {} to ros topic {} with a frequency of {}Hz'.format(message, publisher.name, sending_rate))
        #     rate = rospy.Rate(sending_rate)
        #     while not rospy.is_shutdown():
        #         rospy.loginfo(message)
        #         __publish__(pub=publisher, message=message)
        #         rate.sleep()
        # else:
        if helpers.ros_core_is_running():
            debug(0, 'Sending {} to ros topic {} once'.format(message, publisher.name))
            publisher.publish(message)
            rospy.loginfo(message)
        else:
            debug(1, 'Can not publish to ros. Roscore not running')
    except TypeError as e:
        debug(1, 'Can\'t publish to {}: {}'.format(publisher.name, e.message))


def call_service(service_name, service_class, msg):
    """
    Calls a ROS service.
    If this does not response after a while, the service might be blocked...
    :param service_name: ROS service name as string
    :param service_class: ROS data_class (e.g. std_msgs.String)
    :param msg: Message
    :return:
    """
    #Wait for the service to be free
    rospy.wait_for_service(service_name)
    try:
        debug(0, 'Calling ROS service ', service_name)
        service = rospy.ServiceProxy(name=service_name, service_class=service_class)
        if service_class == LedAnim:
            response = service(msg[0], msg[1], msg[2])
        return response
    except rospy.ServiceException as e:
        debug(1, 'ROS service call failed: ', e)

if __name__ == '__main__':
    print DronarchTopics.pub_land.name