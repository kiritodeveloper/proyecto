#!/usr/bin/python
# -*- coding: UTF-8 -*-
from Robot import Robot
from RobotLogger import start_robot_logger


def main():
    try:
        # Initialize Odometry
        initial_pos = [0.2, 0.2, 0]
        robot = Robot(initial_pos)

        # Robot logger
        start_robot_logger(robot.finished, robot, "./out/trayectoria_entrega.csv")

        # 2. launch updateOdometry thread()
        robot.startRobot()

        robot.go(1, 0.2)

        robot.stopRobot()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        #    robot.stopOdometry()
        robot.stopRobot()


if __name__ == "__main__":
    main()
