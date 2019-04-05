from RobotDrawer import plot_log, plot_log_with_map


def main():
    """
    Plot log file
    """
    #plot_log("./path_logs/trayectoria_2.csv")
    plot_log_with_map('./out/trayectoria_entrega.csv', './maps/mapa_debug.txt')

if __name__ == "__main__":
    main()
