#!/usr/bin/python
# -*- coding: UTF-8 -*-
import math

import time

from config_file import is_debug
from Robot import Robot
from RobotDrawer import start_robot_drawer
from RobotLogger import start_robot_logger

# Queue defined for communication with RobotDrawer
from utils import delay_until


def wait_for_th(th, robot, th_error_margin):
    """
    Wait until the robot reaches the position
    :param robot: robot configuration
    :param th_error_margin: error allowed in the orientation
    """
    [_, _, th_odo] = robot.readOdometry()

    t_next_period = time.time()

    # Repeat while error decrease
    last_error = abs(robot.normalizeAngle(th - th_odo))
    actual_error = last_error
    while th_error_margin < actual_error:
        last_error = actual_error
        while last_error >= actual_error:
            [_, _, th_odo] = robot.readOdometry()
            last_error = actual_error
            actual_error = abs(robot.normalizeAngle(th - th_odo))
            t_next_period += robot.P
            delay_until(t_next_period)


# Odometry paths tests
def path_90_degree_odometry(robot):
    """
    Instructions to do a 90 degrees turn based on odometry
    :param robot: robot configuration
    """
    robot.setSpeed(0, -math.pi / 8)
    wait_for_th(- math.pi / 2, robot, 0.02)

    [_, _, th_odo] = robot.readOdometry()
    print (th_odo)

def main():
    """
    Main function
    """
    try:
        # Instantiate odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory
        path_90_degree_odometry(robot)

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":
    main()
