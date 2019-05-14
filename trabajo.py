#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time
import math
import sys

from TimeUtils import delay_until

from MapLib import Map2D
from Robot import Robot
from RobotLogger import start_robot_logger

# PHASES
# 1 -> DETECTAR SALIDA
# 2 -> SLALOM
# 3 -> LABERINTO
# 4 -> COGER PELOTA
# 5 -> SALIR
salida = 'A'
sizeCell = 400  # in mm

# DUBUG PHASES
phase_from = 1
phase_to = 5


def coord2Meters(coord):
    new_coord = [None, None, None]
    new_coord[0] = (coord[0] + 0.5) * sizeCell / 1000.0
    new_coord[1] = (coord[1] + 0.5) * sizeCell / 1000.0
    new_coord[2] = coord[2]
    return new_coord


def main(args):
    primera = True
    robot = Robot()

    try:
        # 1. load map and compute costs and path
        # COLOR -> FASE 1
        robot.startRobot()

        # Robot logger
        start_robot_logger(robot.finished, robot, "./out/trayectoria.csv")

        if phase_from <= 1 <= phase_to:
            new_color = robot.detectColor()
            salida = 'A' if new_color == 0 else 'B'
            print('LA SALIDA ES: ', salida)
        else:
            salida = args.mapfile

        map_file = "./maps/mapaA.txt" if salida is "A" else "./maps/mapaB.txt"

        myMap = Map2D(map_file)

        print("Pulsa un botón para empezar")
        sys.stdin.read(1)

        # SLALOM -> FASE 2
        if phase_from <= 2 <= phase_to:
            if salida is 'A':
                starting_point = coord2Meters((1, 7, -math.pi / 2))
                pos1 = (starting_point[0], starting_point[1], -2.677945048)
                pos2 = coord2Meters((1, 5, -0.4636475288))
                pos3 = coord2Meters((1, 3, -2.677945048))
                v = 0.198052943
                w_parado = -math.pi / 8
                w_movimiento = 0.442859844
            else:  # Salida es B
                starting_point = coord2Meters((5, 7, -math.pi / 2))
                pos1 = (starting_point[0], starting_point[1], -0.4636475288)
                pos2 = coord2Meters((5, 5, -2.677945048))
                pos3 = coord2Meters((5, 3, -0.4636475288))
                v = 0.198052943
                w_parado = math.pi / 8
                w_movimiento = -0.442859844

            robot.setOdometry(starting_point[0], starting_point[1], starting_point[2])
            primera = False

            # girar 90
            robot.setSpeed(0, w_parado)
            robot.wait_for_th(pos1[2], 0.02)

            # semicirculo 1
            robot.setSpeed(v, w_movimiento)
            robot.wait_for_position(pos2[0], pos2[1], 0.2, True)

            # semicirculo 2
            robot.setSpeed(v, -w_movimiento)
            robot.wait_for_position(pos3[0], pos3[1], 0.2, True)

            # Me detengo
            robot.setSpeed(0, 0)

        # LABERINTO -> FASE 3
        if phase_from <= 3 <= phase_to:
            if salida is 'A':
                starting_point = coord2Meters((1, 3, -math.pi / 2))
                init_pos = [1, 3]
                goal_x = 3
                goal_y = 3
            else:  # Salida es B
                starting_point = coord2Meters((5, 3, -math.pi / 2))
                init_pos = [5, 3]
                goal_x = 3
                goal_y = 3

            if primera:
                robot.setOdometry(starting_point[0], starting_point[1], starting_point[2])

            # Enable sensors
            robot.enableProximitySensor(True)

            myMap.fillCostMatrix([goal_x, goal_y])
            route = myMap.planPath([init_pos[0], init_pos[1]], [goal_x, goal_y])

            robot_locations = []

            last_reached_pos = [init_pos[0], init_pos[1]]

            while len(route) > 0:
                goal = route.pop(0)

                partial_goal_x = (goal[0] + 0.5) * myMap.sizeCell / 1000.0
                partial_goal_y = (goal[1] + 0.5) * myMap.sizeCell / 1000.0

                no_obstacle = robot.go(partial_goal_x, partial_goal_y)
                x_odo, y_odo, th_odo = robot.readOdometry()
                if not no_obstacle:
                    # There are a obstacle
                    print('Obstacle detected')
                    x, y, th = myMap.odometry2Cells(x_odo, y_odo, th_odo)

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

            [x, y, th] = robot.readOdometry()
            print("Estoy principio 4", x, y, th)

        # COGER PELOTA -> FASE 4
        if phase_from <= 4 <= phase_to:
            # Disable sensors
            robot.enableProximitySensor(False)

            redMin = (168, 180, 80)
            redMax = (2, 255, 255)

            res = robot.trackObject(salida, colorRangeMin=redMin, colorRangeMax=redMax)

            print('Espero a que la camara se apague')
            time.sleep(0.2)  # espera en segundos
            print('Supongo que la camara esta apagada')

        # SALIDA -> FASE 5
        [x, y, th] = robot.readOdometry()
        print("Principio de la 5", x, y, th)

        if phase_from <= 5 <= phase_to:
            last_pos = coord2Meters([3, 7, 0])
            robot.go(last_pos[0], last_pos[1])
            robot.orientate(math.pi / 2)
            robot.setSpeed(0.3, 0)
            time.sleep(1)
            robot.setSpeed(0, 0)
        robot.stopRobot()

    except KeyboardInterrupt:
        robot.catch("up")
        robot.stopRobot()


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="B")
    args = parser.parse_args()
    main(args)
