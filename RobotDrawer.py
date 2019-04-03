import csv
from multiprocessing import Process

from MapLib import Map2D
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


def plot_log_with_map(file_path, map_path):
    """
    Read a path log file and plot it
    :param file_path: Path log file path
    """
    myMap = Map2D(map_path)

    robot_locations = []

    with open(file_path) as log:
        i = csv.reader(log)
        for line in i:
            x = float(line[0])
            y = float(line[1])
            th = float(line[2])
            robot_locations = robot_locations + [[int(x* 1000), int(y * 1000), int(th * 1000)]]

    myMap.drawMapWithRobotLocations(robot_locations)

    myMap.stopMap()


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
    plt.ion()

    # Wait until update odometry start
    plt.pause(0.5)

    while not finished.value:
        [x, y, th] = robot.readOdometry()
        dibrobot([x, y, th], 'r', 'p')
        plt.pause(0.3)
