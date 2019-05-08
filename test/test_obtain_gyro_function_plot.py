import csv
import numpy as np

import matplotlib.pyplot as plt

file_path = "../resultados_87_dg.txt"
period = 0.03

data = []

with open(file_path) as log:
    i = csv.reader(log)
    for line in i:
        data.append([float(line[0]), int(line[1]), int(line[2])])

last_values_odometry = [0, 0, 0, 0, 0]

last_values_gyro_1 = [0, 0, 0, 0, 0]

last_values_gyro_2 = [0, 0, 0, 0, 0]

data_normalized = []

data_normalized_p = []

data_normalized_gyro_1 = []

data_normalized_gyro_1_p = []

data_normalized_gyro_2 = []

data_normalized_gyro_2_p = []

for i in data:
    # Odometria
    last_values_odometry.pop(0)
    last_values_odometry.append(i[0])
    actual_value = sum(last_values_odometry) / len(last_values_odometry)
    actual_value_median = np.percentile(last_values_odometry, 50)
    data_normalized.append(actual_value)
    data_normalized_p.append(actual_value_median)

    # Sensor 1
    last_values_gyro_1.pop(0)
    last_values_gyro_1.append(i[1])
    actual_value = sum(last_values_gyro_1) / len(last_values_gyro_1)
    actual_value_median = np.percentile(last_values_gyro_1, 50)

    data_normalized_gyro_1.append(actual_value)
    data_normalized_gyro_1_p.append(actual_value_median)

    # Sensor 2
    last_values_gyro_2.pop(0)
    last_values_gyro_2.append(i[2])
    actual_value = sum(last_values_gyro_2) / len(last_values_gyro_2)
    actual_value_median = np.percentile(last_values_gyro_2, 50)

    data_normalized_gyro_2.append(actual_value)
    data_normalized_gyro_2_p.append(actual_value_median)


plt.plot(list(map(lambda a: a[0], data)), 'ro')
plt.ylabel('Odometry from wheels')
plt.show()


plt.plot(data_normalized, 'ro')
plt.ylabel('Odometry from wheels media')
plt.show()

plt.plot(data_normalized_p, 'ro')
plt.ylabel('Odometry from wheels mediana')
plt.show()

"""

plt.plot(list(map(lambda a: a[1], data)), 'ro')
plt.ylabel('Odometry from gyro 1')
plt.show()

"""
plt.plot(data_normalized_gyro_1, 'ro')
plt.ylabel('Odometry from gyro 1 media')
plt.show()


"""
plt.plot(data_normalized_gyro_1_p, 'ro')
plt.ylabel('Odometry from gyro 1 mediana')
plt.show()
"""

"""
plt.plot(list(map(lambda a: a[2], data)), 'ro')
plt.ylabel('Odometry from gyro 2')
plt.show()
"""

plt.plot(data_normalized_gyro_2, 'ro')
plt.ylabel('Odometry from gyro 2 media')
plt.show()

"""
plt.plot(data_normalized_gyro_2_p, 'ro')
plt.ylabel('Odometry from gyro 2 mediana')
plt.show()
"""