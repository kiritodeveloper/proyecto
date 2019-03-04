from Robot import Robot
from RobotDrawer import start_robot_drawer
from RobotDrawer import plot_log

def main():

    robot = Robot()
    start_robot_drawer(robot.finished, robot)

    plot_log("/home/fernando/Escritorio/Robotica/practica_2/trayectoria_1.csv", robot.finished)