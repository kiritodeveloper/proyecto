#!/usr/bin/python
# -*- coding: UTF-8 -*-
import math
import os

import matplotlib
from config_file import is_debug


matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
import argparse
from Robot import Robot
from RobotLogger import start_robot_logger
from RobotDrawer import *


# NOTES ABOUT TASKS to DO in P4:
# 1)findPath(x1,y1, x2,y2),   fillCostMatrix(), replanPath () --> should be methods from the new Map2D class
# 2) go(x,y) and detectObstacle() could be part of your Robot class (depending how you have implemented things)
# 3) you can change the method signatures if you need, depending how you have implemented things


def main(args):
    """
    Example to load "mapa1.txt"
    """

    try:
        robot = Robot()
        robot.startOdometry()


        # Detect color
        '''color = None
        while True:
            new_color = robot.detect_color()
            if color != new_color:
                print("Color obtenido: "+str(new_color))
                color = new_color


        if color == "Black":
            direction = 1
        else:
            direction = -1'''
        while True:
            robot.turn_off_color()

        robot.stopOdometry()


    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        #    robot.stopOdometry()
        robot.stopOdometry()


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="./maps/mapa_debug.txt")
    args = parser.parse_args()
    main(args)
