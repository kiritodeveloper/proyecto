#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
import math
import cv2

from RobotDrawer import start_robot_drawer
from utils import delay_until


#import matplotlib
from config_file import is_debug


#matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from MapLib import Map2D
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

# LOGO -> BB8 - R2D2

logo = 'R2D2'

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

    primera = True

    try:
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)

        # TODO START ODOMETRY POR SEPARADO

        # SLALOM -> FASE 2

        if phase_from <= 2 and 2 <= phase_to:
            primera = False
            if salida is 'A':
                starting_point = coord2Meters((1, 7, -math.pi/2))
                pos1 = (starting_point[0], starting_point[1], math.pi)
                pos2 = coord2Meters((1, 5, 0))
                pos3 = coord2Meters((1, 3, math.pi))
                pos4 = coord2Meters((1, 3, -math.pi/2))
                v = 0.2
                w_movimiento = 0.50
            else: # Salida es B
                starting_point = coord2Meters((5, 7, -math.pi/2))
                pos1 = (starting_point[0], starting_point[1], 0)
                pos2 = coord2Meters((5, 5, math.pi))
                pos3 = coord2Meters((5, 3, 0))
                pos4 = coord2Meters((5, 3, -math.pi/2))
                v = 0.2
                w_movimiento = -0.50

            robot = Robot(starting_point)
            # Robot logger
            start_robot_logger(robot.finished, robot, "./out/trayectoria_trabajo.csv")
            robot.startOdometry()

            # girar 90
            robot.orientate(pos1[2])

            # semicirculo 1
            robot.setSpeed(v, w_movimiento)
            robot.wait_for_position(pos2[0], pos2[1], robot, 0.2)

            # semicirculo 2
            robot.setSpeed(v, -w_movimiento)
            robot.wait_for_position(pos3[0], pos3[1], robot, 0.2)

            # Giro 90 grados mirando al frente
            robot.setSpeed(0, 0)

            robot.orientate(pos4[2])

        # LABERINTO -> FASE 3

        if phase_from <= 3 and 3 <= phase_to:
            if salida is 'A':
                starting_point = coord2Meters((1, 3, -math.pi/2))
                init_pos = [1, 3]
                goal_x = 3
                goal_y = 2
            else:  # Salida es B
                starting_point = coord2Meters((5, 3, -math.pi/2))
                init_pos = [3, 2]
                goal_x = 3
                goal_y = 2

            if primera:
                robot = Robot(starting_point)
                # Robot logger
                start_robot_logger(robot.finished, robot, "./out/trayectoria_trabajo.csv")
                robot.startOdometry()

            primera = False

            print("Salida: ", salida)
            myMap.fillCostMatrix([goal_x, goal_y])
            route = myMap.planPath([init_pos[0], init_pos[1]], [goal_x, goal_y])

            robot_locations = []

            # TODO is debug poner que dibuje

            last_reached_pos = [init_pos[0], init_pos[1]]

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


            # ORIENTARSE Y AVANZAR UN POCO PARA DELANTE
            # Avanza un poco hacia delante para cruzar la linea de meta
            robot.orientate(math.pi / 2)
            robot.setSpeed(0.2, 0)
            time.sleep(2)
            robot.setSpeed(0, 0)


        # COGER PELOTA -> FASE 4

        if phase_from <= 4 and 4 <= phase_to:
            if primera:
                if salida is 'A':
                    robot = Robot(coord2Meters([3, 3, math.pi/2]))
                else:
                    robot = Robot(coord2Meters([3, 3, math.pi/2]))

                if is_debug:
                    start_robot_drawer(robot.finished, robot)
                else:
                    start_robot_logger(robot.finished, robot, "trayectoria_tracking.csv")

                # 1. launch updateOdometry thread()
                robot.startOdometry()

            primera = False


            redMin = (168, 180, 80)
            redMax = (2, 255, 255)

            res = robot.trackObject(salida, colorRangeMin=redMin, colorRangeMax=redMax)

            print('Espero a que la camara se apague')
            time.sleep(3)  # espera en segundos
            print('Supongo que la camara esta apagada')


        # RECONOCIMIENTO -> FASE 5
        if phase_from <= 5 and 5 <= phase_to:
            # TODO si es la primera activar odometria y demas
            # NO PUEDE SER LA PRIEMRA FASE, TIENE QUE COGER PELOTA PRIMERO
            reco = Reco()

            if salida is 'A':
                cell_to_recognize = coord2Meters([4, 6, 0])
                cell_to_exit_left = coord2Meters([3, 7, 0])
                cell_to_exit_right = coord2Meters([6, 7, 0])
            else:
                cell_to_recognize = coord2Meters([2, 6, 0])
                cell_to_exit_left = coord2Meters([0, 7, 0])
                cell_to_exit_right = coord2Meters([3, 7, 0])

            robot.go(cell_to_recognize[0], cell_to_recognize[1])

            # ORIENTARSE HACIA ARRIBA (mirando al frente)

            robot.orientate(math.pi/2)

            R2D2 = cv2.imread("reco/R2-D2_s.png", cv2.IMREAD_COLOR)
            BB8 = cv2.imread("reco/BB8_s.png", cv2.IMREAD_COLOR)
            R2D2_detected = False
            BB8_detected = False

            R2D2_detected, R2D2_points = reco.search_img(R2D2)
            BB8_detected, BB8_points = reco.search_img(BB8)

            print(R2D2_detected, BB8_detected)

            # SALIR POR LA PUERTA CORRESPONDIENTE
            if BB8_detected and logo == 'BB8' and salida == 'A':
                print('1')
                robot.go(cell_to_exit_left[0], cell_to_exit_left[1])
            elif R2D2_detected and logo == 'BB8' and salida == 'A':
                print('2')
                robot.go(cell_to_exit_right[0], cell_to_exit_right[1])
            elif R2D2_detected and logo == 'R2D2' and salida == 'A':
                print('3')
                robot.go(cell_to_exit_left[0], cell_to_exit_left[1])
            elif BB8_detected and logo == 'R2D2' and salida == 'A':
                print('4')
                robot.go(cell_to_exit_right[0], cell_to_exit_right[1])


            # Avanza un poco hacia delante para cruzar la linea de meta
            robot.orientate(math.pi/2)
            robot.setSpeed(0.2, 0)
            time.sleep(2)
            robot.setSpeed(0, 0)


        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        #    robot.stopOdometry()
        robot.catch("up")
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
