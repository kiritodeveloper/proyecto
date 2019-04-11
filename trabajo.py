#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
import math
import cv2

#import matplotlib
from config_file import is_debug


#matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from Robot import Robot
from RobotLogger import start_robot_logger

# Recognization
from reco import Reco


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

        robot = Robot()
        reco = Reco()

        # Robot logger
        start_robot_logger(robot.finished, robot, "./out/trayectoria_entrega.csv")

        robot.startOdometry()


        # ------ RECONOCIMIENTO ------

        R2D2 = "reco/R2-D2_s.png"
        BB8 = "reco/BB8_s.png"
        R2D2_detected = False
        BB8_detected = False

        robot.setSpeed(0, math.pi / 16)

        while not R2D2_detected or not BB8_detected:
            if not R2D2_detected:
                R2D2_detected, R2D2_points = reco.search_img(R2D2)
                if R2D2_detected:
                    R2D2_th = robot.readOdometry()[2]
                cv2.waitKey(1) # Time beteween frames
            if not BB8_detected:
                BB8_detected, BB8_points = reco.search_img(BB8)
                if BB8_detected:
                    BB8_th = robot.readOdometry()[2]
                cv2.waitKey(1)
            print(R2D2_detected, BB8_detected)

        print(R2D2_th, BB8_th)
        R2D2_pos, BB8_pos = reco.get_orientation(R2D2_th, BB8_th)
        print('R2D2: ',R2D2_pos, ' BB8: ', BB8_pos)

        robot.setSpeed(0, 0)

        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        #    robot.stopOdometry()
        robot.stopOdometry()
        reco.stop_camera()


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="./maps/mapa_debug.txt")
    args = parser.parse_args()
    main(args)
