#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import math

import numpy as np
import time

from Robot import Robot


def trayectoria_1_tiempos(robot):
    robot.setSpeed(0, -math.pi / 8)
    time.sleep(4)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    time.sleep(16)

    robot.setSpeed(math.pi / 16, -math.pi / 16)
    time.sleep(32)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    time.sleep(16)


def espera_posicion(x, y, th, robot):
    [x_odo, y_odo, th_odo] = robot.startOdometry()
    margen_error = 0.02

    margen_error_th = 0.1

    while (margen_error > math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)) & (margen_error_th > abs(th - th_odo)):
        print(th_odo)
        [x_odo, y_odo, th_odo] = robot.startOdometry()


def trayectoria_90_grados_odometria(robot):
    robot.setSpeed(0, -math.pi / 8)
    espera_posicion(0, 0, math.pi / 2, robot)


def trayectoria_1_m_odometria(robot):
    robot.setSpeed(0.1, -math.pi / 8)
    espera_posicion(0.8, 0, 0, robot)


def trayectoria_1_odometria(robot):
    robot.setSpeed(0, -math.pi / 8)
    espera_posicion(0, 0, - math.pi / 2, robot)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    espera_posicion(2, 0, math.pi / 2, robot)

    robot.setSpeed(math.pi / 16, -math.pi / 16)
    espera_posicion(4, 0, -math.pi / 2, robot)

    espera_posicion(2, 0, math.pi / 2, robot)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    espera_posicion(0, 0, - math.pi / 2, robot)


def trayectoria_2_tiempos(robot):
    robot.setSpeed(0, math.pi / 8)
    time.sleep(4)

    robot.setSpeed(math.pi / 18, - math.pi / 9)
    time.sleep(3)

    robot.setSpeed(0.1, 0)
    time.sleep(10)

    robot.setSpeed(math.pi / 15, - math.pi / 15)
    time.sleep(20)

    robot.setSpeed(0.1, 0)
    time.sleep(10)

    robot.setSpeed(math.pi / 18, - math.pi / 9)
    time.sleep(3)


def trayectoria_2_odometria(robot):
    robot.setSpeed(0, math.pi / 8)
    time.sleep(4)

    robot.setSpeed(math.pi / 18, - math.pi / 9)
    time.sleep(3)

    robot.setSpeed(0.1, 0)
    time.sleep(10)

    robot.setSpeed(math.pi / 15, - math.pi / 15)
    time.sleep(20)

    robot.setSpeed(0.1, 0)
    time.sleep(10)

    robot.setSpeed(math.pi / 18, - math.pi / 9)
    time.sleep(3)


def main(args):
    try:
        '''  if args.radioD < 0:
            print 'd must be a positive value'
            exit(1)
        '''
        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %d" % (robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory

        trayectoria_90_grados_odometria(robot)
        '''
        # DUMMY CODE! delete when you have your own
        robot.setSpeed(1,0)
        print "Start : %s" % time.ctime()
        time.sleep(3)
        print("X value from main tmp %d" % robot.x.value)
        time.sleep(3)
        print "End : %s" % time.ctime()

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %d, %d, %d" % (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # PART 1:
        # robot.setSpeed()
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...

        '''

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":
    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)
