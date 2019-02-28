import time
from multiprocessing import Process

from plot_robot import dibrobot
import matplotlib.pyplot as plt



def start_robot_drawer(finished, robot):
    p = Process(target=loop_robot_drawer, args=(finished, robot))
    p.start()

    # Time to start drawer
    print("Init drawer")
    time.sleep(10)

def loop_robot_drawer(finished, robot):
    print("Drawer started")
    dibrobot([0, 0, 0], 'r', 'p')
    plt.ion()

    time.sleep(10)

    while not finished.value:
        [x, y, th] = robot.readOdometry()
        dibrobot([x, y, th], 'r', 'p')
        plt.pause(0.3)
