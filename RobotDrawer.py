import time
from multiprocessing import Value, Process, Queue

from plot_robot import dibrobot
import matplotlib.pyplot as plt


def start_robot_drawer(finished: Value, period: float, child_conn: Queue):
    p: Process = Process(target=loop_robot_drawer, args=(finished, period, child_conn))
    p.start()

    # Time to start drawer
    print("Init drawer")
    time.sleep(10)


def loop_robot_drawer(finished: Value, period: float, child_conn: Queue):
    print("Drawer started")
    dibrobot([0, 0, 0], 'r', 'p')
    plt.ion()

    time.sleep(10)

    while not finished.value or not child_conn.empty():
        [x, y, th] = child_conn.get()
        dibrobot([x, y, th], 'r', 'p')
        plt.pause(period)

    child_conn.close()
