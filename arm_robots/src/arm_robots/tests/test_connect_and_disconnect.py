from random import random
from time import sleep

import numpy as np
from threading import Thread

from arc_utilities import ros_init
from arm_robots.hdt_michigan import Val
from link_bot_gazebo_python.gazebo_services import GazeboServices


def mess_with_sim_time():
    service_provider = GazeboServices()
    rng = np.random.RandomState()
    for i in range(100):
        r = rng.uniform()
        if r > 0.8:
            service_provider.play()
        elif r > 0.6:
            service_provider.pause()
        sleep(0.5)


@ros_init.with_ros("test_connect_and_disconnect")
def main():
    t = Thread(target=mess_with_sim_time)
    # t.start()

    for i in range(100):
        robot = Val()
        robot.connect()
        robot.disconnect()

        robot.connect()
        robot.disconnect()

        robot.connect()
        robot.connect()
        robot.disconnect()
        robot.disconnect()
        robot.connect()
        robot.disconnect()

    t.join()

if __name__ == '__main__':
    main()
