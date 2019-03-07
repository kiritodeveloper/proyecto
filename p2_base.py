#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import math

import time

from config_file import is_debug
from Robot import Robot
from RobotDrawer import start_robot_drawer
from RobotLogger import start_robot_logger

# Queue defined for communication with RobotDrawer
from utils import delay_until


# Timed paths
def path_1_timed(robot):
    robot.setSpeed(0, -math.pi / 8)
    time.sleep(4)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    time.sleep(16)

    robot.setSpeed(math.pi / 16, -math.pi / 16)
    time.sleep(32)

    robot.setSpeed(math.pi / 16, math.pi / 16)
    time.sleep(16)


# Odometry paths tests
def path_90_degree_odometry(robot):
    robot.setSpeed(0, -math.pi / 8)
    wait_for_position(0, 0, - math.pi / 2, robot, 0.2, 0.02)


def path_1_m_odometry(robot):
    robot.setSpeed(0.1, 0)
    wait_for_position(0.8, 0, 0, robot, 0.2, 0.02)


# Odometry paths
def path_1_odometry(robot):
    robot.setSpeed(0, -math.pi / 8)
    wait_for_position(0, 0, - math.pi / 2, robot, 0.2, 0.02)

    robot.setSpeed(0.2, 0.5)
    wait_for_position(0.8, 0, math.pi / 2, robot, 0.2, 0.02)

    robot.setSpeed(0.2, -0.5)
    wait_for_position(1.6, 0, -math.pi / 2, robot, 0.2, 0.02)

    wait_for_position(0.8, 0, math.pi / 2, robot, 0.2, 0.02)

    robot.setSpeed(0.2, 0.5)
    wait_for_position(0, 0, - math.pi / 2, robot, 0.2, 0.02)


def path_2_odometry(robot):
    robot.setSpeed(0, math.pi / 8)
    wait_for_position(0, 0, math.pi / 2, robot, 0.01, 0.02)

    robot.setSpeed(0.2, -0.4)
    wait_for_position(0.38, 0.48, 0.28379410920832804, robot, 0.05,
                      0.02)  # th= math.pi/2 - math.asin(0.236/0.3)

    robot.setSpeed(0.2, 0)
    wait_for_position(1.88, 0.87,  0.28379410920832804, robot, 0.1,
                      0.02)  # th= math.pi/2 - math.asin(0.236/0.3)

    robot.setSpeed(0.2, - 0.2222222222222222)
    wait_for_position(1.88, -0.87, 2.882671111583572, robot, 0.15,
                      0.02)  # th = math.pi/2 + math.asin(0.586/0.6)

    robot.setSpeed(0.2, 0)
    wait_for_position(0.38, -0.48, 2.882671111583572, robot, 0.2,
                      0.02)  # th = math.pi/2 + math.asin(0.586/0.6)

    robot.setSpeed(0.2, -0.4)
    wait_for_position(0, 0, math.pi / 2, robot, 0.3, 0.02)


def wait_for_position(x, y, th, robot, position_error_margin, th_error_margin):
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

        start_robot_logger(robot.finished, robot, "/home/pi/trayectoria_1.csv")

        print("X value at the beginning from main X= %d" % (robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory
        path_2_odometry(robot)

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
