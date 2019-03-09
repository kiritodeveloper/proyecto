import time
from multiprocessing import Process


def start_robot_logger(finished, robot, file_name):
    p = Process(target=loop_robot_logger, args=(finished, robot, file_name))
    p.start()


def loop_robot_logger(finished, robot, file_name):
    print("Logger started")

    file_to_write = open(file_name, "w")
    file_to_write.write("0,0,0\n")

    # Wait until update odometry start
    time.sleep(0.5)

    while not finished.value:
        [x, y, th] = robot.readOdometry()
        line_to_write = str(x) + "," + str(y) + "," + str(th) + "\n"
        file_to_write.write(line_to_write)
        time.sleep(0.3)

    file_to_write.close()
