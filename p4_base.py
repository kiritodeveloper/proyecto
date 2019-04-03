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
from RobotDrawer import *

from utils import delay_until

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
        initial_pos = [0.4, 0.4, 0]
        robot = Robot(initial_pos)

        # this will open a window with the results, but does not work well remotely

        #  sampleRobotLocations = [[200, 200, 3.14 / 2.0], [200, 600, 3.14 / 4.0], [200, 1000, -3.14 / 2.0], ]
        # myMap.drawMapWithRobotLocations(sampleRobotLocations)

        # 2. launch updateOdometry thread()
        robot.startOdometry()

        goal_x = 3
        goal_y = 0


        myMap.fillCostMatrix([goal_x, goal_y])
        route = myMap.planPath([0, 0], [goal_x, goal_y])

        RobotLocations = []

        #myMap.drawMap(saveSnapshot=False)
        #plt.show()
        start_robot_drawer(robot.finished, robot)
        last_reached_pos = None

        finished = False
        while not finished:
            for goal in route:
                print('Ruta', route)
                partial_goal_x = (goal[0] + 1.0) * myMap.sizeCell/1000.0
                partial_goal_y = (goal[1] + 1.0) * myMap.sizeCell/1000.0
                print('Partials: ', partial_goal_x, partial_goal_y)
                print('El goal: ', goal)
                print('Estoy: ', robot.readOdometry())
                reached = robot.go(partial_goal_x, partial_goal_y, myMap)
                if not reached:
                    print('NO HA ALCANZADO EL OBJETIVO')
                    route = myMap.replanPath(goal_x, goal_y)
                    break
                else:
                    RobotLocations.append([myMap.pos_x, myMap.pos_y, myMap.pos_th])
                    last_reached_pos = goal

            if last_reached_pos[0] == goal_x and last_reached_pos[1] == goal_y:
                finished = True

        myMap.drawMapWithRobotLocations(RobotLocations)
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
                        default="./maps/mapa2.txt")
    args = parser.parse_args()
    main(args)
