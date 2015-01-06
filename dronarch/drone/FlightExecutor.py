__author__ = 'niclas'

from time import sleep

# from ArdroneCoordinator import ArdroneCoordinator

class FlightExecutor:

    def __init__(self, interval):
        # self.ardrone = ArdroneCoordinator()
        self.interval = interval

    def execute_flight(self, flight_iter):
        for command in flight_iter:
            # self.ardrone.execute_command(command)
            print command
            sleep(self.interval)

    def flight_iterator(self):
        while True:
            yield 'bla'

if __name__=='__main__':
    exe = FlightExecutor(0.5)
    fly = exe.flight_iterator()
    exe.execute_flight(flight_iter=fly)