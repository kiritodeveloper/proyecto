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



# NOTES ABOUT TASKS to DO in P4:
# 1)findPath(x1,y1, x2,y2),   fillCostMatrix(), replanPath () --> should be methods from the new Map2D class
# 2) go(x,y) and detectObstacle() could be part of your Robot class (depending how you have implemented things)
# 3) you can change the method signatures if you need, depending how you have implemented things


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
            time.sleep(4)
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
