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


def main():
    """
    Main function
    """
    try:
        # Instantiate odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        if is_debug:
            start_robot_drawer(robot.finished, robot)
        else:
            start_robot_logger(robot.finished, robot, "./out/trayectoria_1.csv")

        print("X value at the beginning from main X= %d" % (robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory
        robot.go(2.5, 1)

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
