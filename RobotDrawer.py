import csv
from multiprocessing import Process

from plot_robot import dibrobot
import matplotlib.pyplot as plt


def plot_log(file_path):
    """
    Read a path log file and plot it
    :param file_path: Path log file path
    """
    with open(file_path) as log:
        i = csv.reader(log)
        for line in i:
            dibrobot([float(line[0]), float(line[1]), float(line[2])], 'r', 'p')
    plt.show()


def start_robot_drawer(finished, robot):
    """
    Start robot drawer process, it should be loaded only in debug mode to watch the robot track simulation
    :param finished: if finish is true, must stop updating position
    :param robot: Robot object
    """
    p = Process(target=loop_robot_drawer, args=(finished, robot))
    p.start()


def loop_robot_drawer(finished, robot):
    """
    Loop that plot the robot position every 0.3 seconds
    :param finished: if finish is true, must stop updating position
    :param robot: Robot object
    """
    print("Drawer started")
    dibrobot([0, 0, 0], 'r', 'p')
    plt.ion()

    # Wait until update odometry start
    plt.pause(0.5)

    while not finished.value:
        [x, y, th] = robot.readOdometry()
        dibrobot([x, y, th], 'r', 'p')
        plt.pause(0.3)
