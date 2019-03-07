import time
from multiprocessing import Process


def start_robot_logger(finished, robot, file_name):
    p = Process(target=loop_robot_logger, args=(finished, robot, file_name))
    p.start()

    # Time to start drawer
    print("Init logger")


def loop_robot_logger(finished, robot, file_name):
    print("Logger started")

    file = open(file_name, "w")
    file.write("0,0,0\n")

    while not finished.value:
        [x, y, th] = robot.readOdometry()
        lineToWrite = str(x) + "," + str(y) + "," + str(th) + "\n"
        file.write(lineToWrite)
        time.sleep(0.3)

    file.close()
