from RobotDrawer import plot_log, plot_log_with_map


def main():
    """
    Plot log file
    """
    plot_log("./out/trayectoria_entrega.csv")
    #plot_log_with_map('./path_logs/trayectoria_mapa3_debug.csv', './maps/mapa3.txt')


if __name__ == "__main__":
    main()
