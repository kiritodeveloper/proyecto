import time
from multiprocessing import Value, Process

from plot_robot import dibrobot
import matplotlib.pyplot as plt


def start_robot_drawer(finished: Value, period: float, child_conn):
    p: Process = Process(target=loop_robot_drawer, args=(finished, period, child_conn))
    p.start()

    # Time to start drawer
    time.sleep(4)


def loop_robot_drawer(finished: Value, period: float, child_conn):
    dibrobot([0, 0, 0], 'r', 'p')
    plt.ion()

    time.sleep(10)

    while not finished.value:
        [x, y, th] = child_conn.recv()
        dibrobot([x, y, th], 'r', 'p')
        plt.pause(period)

    child_conn.close()
