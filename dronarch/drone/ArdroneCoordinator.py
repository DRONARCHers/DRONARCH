__author__ = 'niclas'
from dronarch.sfm.Dronarch import Dronarch
from RosInterface import RosInterface

class ArdroneCoordinator:

    coord = None
    def __init__(self):
        self.dron = Dronarch.get_instance()
        self.ros = RosInterface.get_instance()

    @classmethod
    def get_instance(self):
        if ArdroneCoordinator.coord == None:
            ArdroneCoordinator.coord = ArdroneCoordinator()
        return ArdroneCoordinator.coord

    def start_ros(self):
        self.ros.start_ros()

    def stop_ros(self):
        self.ros.stop_ros()

    def start_video(self):
        self.ros.start_video(self.dron.vid_imgs_per_sec)

    def stop_video(self):
        self.ros.stop_video()

    def anim_led(self):
        self.ros.anim_led()

    def land(self):
        self.ros.land()