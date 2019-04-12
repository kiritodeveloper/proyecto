#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
import math

import matplotlib
from config_file import is_debug

matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from MapLib import Map2D
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
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)

        # Initialize Odometry
        initial_pos = [0.2, 0.2, 0]
        robot = Robot(initial_pos)

        # Robot logger
        start_robot_logger(robot.finished, robot, "./out/trayectoria_entrega.csv")

        # 2. launch updateOdometry thread()
        robot.startOdometry()

        goal_x = 2
        goal_y = 2

        myMap.fillCostMatrix([goal_x, goal_y])
        route = myMap.planPath([0, 0], [goal_x, goal_y])

        robot_locations = []

        if is_debug:
            start_robot_drawer(robot.finished, robot)
        last_reached_pos = [0, 0]

        while len(route) > 0:
            goal = route.pop(0)
            print('Ruta', route)
            partial_goal_x = (goal[0] + 0.5) * myMap.sizeCell / 1000.0
            partial_goal_y = (goal[1] + 0.5) * myMap.sizeCell / 1000.0
            print('Partials: ', partial_goal_x, partial_goal_y)
            print('El goal: ', goal)
            print('Estoy: ', robot.readOdometry())
            no_obstacle = robot.go(partial_goal_x, partial_goal_y)
            x_odo, y_odo, th_odo = robot.readOdometry()
            if not no_obstacle:
                # There are a obstacle
                print('Obstacle detected')
                x, y, th = myMap.odometry2Cells(x_odo, y_odo, th_odo)
                print('ODOMETRIIIA:', x, y, th)
                # Delete connections from detected wall
                myMap.deleteConnection(int(x), int(y), myMap.rad2Dir(th))
                myMap.deleteConnection(int(x), int(y), (myMap.rad2Dir(th) + 1) % 8)
                myMap.deleteConnection(int(x), int(y), (myMap.rad2Dir(th) - 1) % 8)

                route = myMap.replanPath(last_reached_pos[0], last_reached_pos[1], goal_x, goal_y)
            else:
                robot_locations.append([int(x_odo * 1000), int(y_odo * 1000), int(th_odo * 1000)])
                last_reached_pos = goal

        if last_reached_pos[0] == goal_x and last_reached_pos[1] == goal_y:
            print('The goal has been reached')
        else:
            print('Can\'t reached the goal')

        myMap.drawMapWithRobotLocations(robot_locations)

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
