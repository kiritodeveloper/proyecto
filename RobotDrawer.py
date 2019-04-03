import csv
from multiprocessing import Process

from plot_robot import dibrobot
import matplotlib.pyplot as plt


def plot_log(file_path):
    """
    Read a path log file and plot it
    :param file_path: Path log file path
    """
    with open(file_path) as log:
        i = csv.reader(log)
        for line in i:
            dibrobot([float(line[0]), float(line[1]), float(line[2])], 'r', 'p')
    plt.show()


def start_robot_drawer(finished, robot, initial_map):
    """
    Start robot drawer process, it should be loaded only in debug mode to watch the robot track simulation
    :param finished: if finish is true, must stop updating position
    :param robot: Robot object
    """
    p = Process(target=loop_robot_drawer, args=(finished, robot, initial_map))
    p.start()


def draw_grid(initial_map):
    """
    aux function to create a grid with map lines
    """
    if not initial_map.current_ax:
        print("Error plotting: do not call this function directly, \
            call drawMap first to create a plot where to draw")
        return False

    plt.rc('grid', linestyle="--", color='gray')
    plt.grid(True)
    plt.tight_layout()

    x_t = range(0, (initial_map.sizeX + 1) * 400, 400)
    y_t = range(0, (initial_map.sizeY + 1) * 400, 400)
    x_labels = [str(n) for n in x_t]
    y_labels = [str(n) for n in y_t]
    plt.xticks(x_t, x_labels)
    plt.yticks(y_t, y_labels)

    # Main rectangle
    X = np.array([0, initial_map.sizeX, initial_map.sizeX, 0, 0]) * initial_map.sizeCell
    Y = np.array([0, 0, initial_map.sizeY, initial_map.sizeY, 0]) * initial_map.sizeCell
    initial_map.current_ax.plot(X, Y, initial_map.mapLineStyle)

    # "vertical" walls
    for i in range(2, 2 * initial_map.sizeX, 2):
        for j in range(1, 2 * initial_map.sizeY, 2):
            if not initial_map.connectionMatrix[i, j]:
                # paint "right" wall from cell (i-1)/2, (j-1)/2
                cx = np.floor((i - 1) / 2)
                cy = np.floor((j - 1) / 2)
                X = np.array([cx + 1, cx + 1]) * initial_map.sizeCell
                Y = np.array([cy, cy + 1]) * initial_map.sizeCell
                initial_map.current_ax.plot(X, Y, initial_map.mapLineStyle)

    # "horizontal" walls
    for j in range(2, 2 * initial_map.sizeY, 2):
        for i in range(1, 2 * initial_map.sizeX, 2):
            if not initial_map.connectionMatrix[i, j]:
                # paint "top" wall from cell (i-1)/2, (j-1)/2
                cx = np.floor((i - 1) / 2)
                cy = np.floor((j - 1) / 2)
                X = np.array([cx, cx + 1]) * initial_map.sizeCell
                Y = np.array([cy + 1, cy + 1]) * initial_map.sizeCell
                initial_map.current_ax.plot(X, Y, initial_map.mapLineStyle)
    plt.axis('equal')

    return True


def loop_robot_drawer(finished, robot, initial_map):
    """
    Loop that plot the robot position every 0.3 seconds
    :param finished: if finish is true, must stop updating position
    :param robot: Robot object
    """
    print("Drawer started")
    plt.ion()

    initial_map._drawGrid()

    # Wait until update odometry start
    plt.pause(0.5)

    while not finished.value:
        [x, y, th] = robot.readOdometry()
        dibrobot([x, y, th], 'r', 'p')
        plt.pause(0.3)
