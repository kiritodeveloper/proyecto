#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
import math
import cv2
from utils import delay_until


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


# ---- PHASES ----
# 1 -> DETECTAR SALIDA
# 2 -> SLALOM
# 3 -> LABERINTO
# 4 -> COGER PELOTA
# 5 -> RECONOCER Y SALIR

salida = 'A'
sizeCell = 400 # in mm

# DUBUG
phase_from = 1
phase_to = 5

def coord2Meters(coord):
    new_coord = [None, None, None]
    new_coord[0] = (coord[0] + 0.5) * sizeCell / 1000.0
    new_coord[1] = (coord[1] + 0.5) * sizeCell / 1000.0
    new_coord[2] = coord[2]
    return new_coord

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

        robot = Robot()
        # reco = Reco()

        # Robot logger
        start_robot_logger(robot.finished, robot, "./out/trayectoria_trabajo.csv")

        # TODO START ODOMETRY POR SEPARADO

        # SLALOM -> FASE 2

        if phase_from <= 2 and 2 <= phase_to:
            if salida is 'A':
                starting_point = coord2Meters((1, 7, -math.pi/2))
                pos1 = (starting_point[0], starting_point[1], math.pi)
                pos2 = coord2Meters((1, 5, math.pi))
                pos3 = coord2Meters((1, 3, - math.pi / 2))
                v = 0.2
                w_parado = -math.pi/8
                w_movimiento = 0.5
            else: # Salida es B
                starting_point = coord2Meters((5, 7, -math.pi/2))
                #pos1 = coord2Meters((5, 5))
                #pos2 = coord2Meters((5, 3))
                v = 0.2
                w_parado = -math.pi / 8
                w_movimiento = 0.5

            robot = Robot(starting_point)
            # Robot logger
            start_robot_logger(robot.finished, robot, "./out/trayectoria_trabajo.csv")
            robot.startOdometry()

            # girar 90
            robot.setSpeed(0, w_parado)
            wait_for_position(pos1[0], pos1[1], pos1[2], robot, 0.2, 0.02)

            # semicirculo 1
            robot.setSpeed(v, w_movimiento)
            wait_for_position(pos2[0], pos2[1], pos2[3], robot, 0.2, 0.02)

            """
            # semicirculo 2
            robot.setSpeed(v, -w_movimiento)
            wait_for_position(1.6, 0, -math.pi / 2, robot, 0.2, 0.02)
            """
            robot.setSpeed(0, 0)

        # ------ RECONOCIMIENTO ------
        """
        R2D2 = cv2.imread("reco/R2-D2_s.png", cv2.IMREAD_COLOR)
        BB8 = cv2.imread("reco/BB8_s.png", cv2.IMREAD_COLOR)
        R2D2_detected = False
        BB8_detected = False

        actual_th = 0
        error = 0.05
        turn_speed = math.pi / 2
        #robot.setSpeed(0, turn_speed)
        #robot.wait_for_th(actual_th, error)

        period = math.pi/4

        while not R2D2_detected or not BB8_detected:
            print('---- COMIENZO BUCLE ----')
            cv2.waitKey(100)  # Time beteween frames
            if not R2D2_detected:
                #cv2.waitKey(1000) # Time beteween frames
                R2D2_detected, R2D2_points = reco.search_img(R2D2)
                if R2D2_detected:
                    R2D2_th = robot.readOdometry()[2]
                    actual_th = robot.normalizeAngle(actual_th+math.pi/4)
                    turn_speed = math.pi/2
                    period = -math.pi/32
            if not BB8_detected:
                #cv2.waitKey(1000) # Time beteween frames
                BB8_detected, BB8_points = reco.search_img(BB8)
                if BB8_detected:
                    BB8_th = robot.readOdometry()[2]
                    actual_th = robot.normalizeAngle(actual_th+math.pi/4)
                    turn_speed = math.pi/2
                    period = -math.pi/32
            print(R2D2_detected, BB8_detected)
            if not R2D2_detected or not BB8_detected:
                actual_th_viejo = actual_th
                actual_th = robot.normalizeAngle(actual_th+period)
                if actual_th_viejo > 5 * math.pi / 6 and actual_th < -math.pi / 4:
                    actual_th_viejo = -actual_th_viejo
                elif actual_th_viejo < -5 * math.pi / 6 and actual_th > math.pi / 4:
                    actual_th_viejo = -actual_th_viejo

                if actual_th_viejo > actual_th:
                    robot.setSpeed(0, -turn_speed)
                    print(actual_th_viejo, actual_th, -turn_speed)
                else:
                    robot.setSpeed(0, turn_speed)
                    print(actual_th_viejo, actual_th, turn_speed)

                robot.wait_for_th(actual_th, error)
                robot.setSpeed(0, 0)

        print(R2D2_th, BB8_th)
        R2D2_pos, BB8_pos = reco.get_orientation(R2D2_th, BB8_th)
        print('R2D2: ',R2D2_pos, ' BB8: ', BB8_pos)

        robot.setSpeed(0, 0)
        """

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
                        default="./maps/mapaA.txt")
    args = parser.parse_args()
    main(args)
