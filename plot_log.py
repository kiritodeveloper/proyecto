from RobotDrawer import plot_log, plot_log_with_map


def main():
    """
    Plot log file
    """
    #plot_log("./path_logs/trayectoria_2.csv")
    plot_log_with_map('./trayectoria_tracking.csv', './maps/mapaA.txt')


if __name__ == "__main__":
    main()
