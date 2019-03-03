#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import math

import time

from config_file import is_debug
from Robot import Robot
from RobotDrawer import start_robot_drawer

# Queue defined for communication with RobotDrawer
from utils import delay_until


def trayectoria_1_tiempos(robot):
    robot.setSpeed(0, -math.pi / 8)
    time.sleep(4)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    time.sleep(16)

    robot.setSpeed(math.pi / 16, -math.pi / 16)
    time.sleep(32)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    time.sleep(16)


def trayectoria_90_grados_odometria(robot):
    robot.setSpeed(0, -math.pi / 8)
    wait_for_position(0, 0, - math.pi / 2, robot)


def trayectoria_1_m_odometria(robot):
    robot.setSpeed(0.1, 0)
    wait_for_position(0.8, 0, 0, robot)


def trayectoria_1_odometria(robot):
    # Plotting odometry
    robot.setSpeed(0, -math.pi / 8)
    wait_for_position(0, 0, - math.pi / 2, robot)

    robot.setSpeed(0.2, 0.5)
    wait_for_position(0.8, 0, math.pi / 2, robot)

    robot.setSpeed(0.2, -0.5)
    wait_for_position(1.6, 0, -math.pi / 2, robot)

    wait_for_position(0.8, 0, math.pi / 2, robot)

    robot.setSpeed(0.2, 0.5)
    wait_for_position(0, 0, - math.pi / 2, robot)


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


def wait_for_position(x, y, th, robot):
    [x_odo, y_odo, th_odo] = robot.readOdometry()

    position_error_margin = 0.2
    th_error_margin = 0.02

    print("Waiting for position ", x_odo, y_odo, th_odo, x, y, th)

    t_next_period = time.time()

    while (position_error_margin < math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)) or (
            th_error_margin < abs(th - th_odo)):
        [x_odo, y_odo, th_odo] = robot.readOdometry()
        t_next_period += robot.P
        delay_until(t_next_period)


def main(args):
    try:
        '''  if args.radioD < 0:
            print 'd must be a positive value'
            exit(1)
        '''
        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        if is_debug:
            start_robot_drawer(robot.finished, robot)

        print("X value at the beginning from main X= %d" % (robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory
        trayectoria_1_odometria(robot)

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

        # Wait until all print is done
        print("Printeando final")

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
