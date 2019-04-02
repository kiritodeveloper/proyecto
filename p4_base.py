#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
import math

import matplotlib

matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from MapLib import Map2D
import argparse
from config_file import is_debug
from Robot import Robot
from RobotDrawer import start_robot_drawer
from RobotLogger import start_robot_logger
import time

from utils import delay_until

# NOTES ABOUT TASKS to DO in P4:
# 1)findPath(x1,y1, x2,y2),   fillCostMatrix(), replanPath () --> should be methods from the new Map2D class
# 2) go(x,y) and detectObstacle() could be part of your Robot class (depending how you have implemented things)
# 3) you can change the method signatures if you need, depending how you have implemented things


def wait_for_position(x, y, th, robot, position_error_margin, th_error_margin):
    """
    Wait until the robot reaches the position
    :param x: x position to be reached
    :param y: y position to be reached
    :param robot: robot configuration
    :param position_error_margin: error allowed in the position
    :param th_error_margin: error allowed in the orientation
    """
    [x_odo, y_odo, th_odo] = robot.readOdometry()

    print("Waiting for position ", x_odo, y_odo, th_odo, x, y, th)

    t_next_period = time.time()

    if th is None:
        print("TH none")
        # None th provided
        while position_error_margin < math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2):
            [x_odo, y_odo, th_odo] = robot.readOdometry()
            t_next_period += robot.P
            delay_until(t_next_period)
    else:
        while (position_error_margin < math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)) or (
                th_error_margin < abs(th - th_odo)):
            [x_odo, y_odo, th_odo] = robot.readOdometry()
            t_next_period += robot.P
            delay_until(t_next_period)
            print ([x_odo, y_odo, th_odo])
    print([x_odo, y_odo, th_odo])


def main(args):
    """
    Example to load "mapa1.txt"
    """

    try:
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile
        # Instantiate Odometry with your own files from P2/P3
        # robot = Robot()
        # ...

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # this will open a window with the results, but does not work well remotely

        #  sampleRobotLocations = [[200, 200, 3.14 / 2.0], [200, 600, 3.14 / 4.0], [200, 1000, -3.14 / 2.0], ]
        # myMap.drawMapWithRobotLocations(sampleRobotLocations)

        # 2. launch updateOdometry thread()
        robot.startOdometry()
        while True:
            myMap.detectObstacle(robot)
            time.sleep(2)
            robot.setSpeed(0, -math.pi / 8)
            wait_for_position(0, 0, - math.pi / 2, robot, 0.2, 0.02)
        # ...

        # 3. perform trajectory
    # robot.setSpeed(1,1) ...
    # while (notfinished){

    # robot.go(pathX[i],pathY[i]);
    # check if there are close obstacles
    # deal with them...
    # Avoid_obstacle(...) OR RePlanPath(...)

    # 4. wrap up and close stuff ...
    # This currently unconfigure the sensors, disable the motors,
    # and restore the LED to the control of the BrickPi3 firmware.
    # robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        #    robot.stopOdometry()
        robot.stopOdometry()
        myMap.stopMap()


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="./maps/mapa1.txt")
    args = parser.parse_args()
    main(args)
