import numpy as np
import matplotlib.pyplot as plt


def dibrobot(loc_eje, c, tamano):
    """
    Plot robot in loc_eje in some color c and with size tamano (p/g)
    :param loc_eje: Location
    :param c: Color
    :param tamano: Size
    """
    if tamano == 'p':
        largo = 0.1
        corto = 0.05
        descentre = 0.01
    else:
        largo = 0.5
        corto = 0.25
        descentre = 0.05

    trasera_dcha = np.array([-largo, -corto, 1])
    trasera_izda = np.array([-largo, corto, 1])
    delantera_dcha = np.array([largo, -corto, 1])
    delantera_izda = np.array([largo, corto, 1])
    frontal_robot = np.array([largo, 0, 1])
    tita = loc_eje[2]
    Hwe = np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
                    [np.sin(tita), np.cos(tita), loc_eje[1]],
                    [0, 0, 1]])
    Hec = np.array([[1, 0, descentre],
                    [0, 1, 0],
                    [0, 0, 1]])
    extremos = np.array(
        [trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
    robot = np.dot(Hwe, np.dot(Hec, np.transpose(extremos)))
    plt.plot(robot[0, :], robot[1, :], c)
