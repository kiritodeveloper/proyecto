from RobotDrawer import plot_log, plot_log_with_map


def main():
    """
    Plot log file
    """
    #plot_log("./path_logs/trayectoria_2.csv")
    plot_log_with_map('./out/trayectoria_1.csv', './maps/mapa3.txt')

if __name__ == "__main__":
    main()
