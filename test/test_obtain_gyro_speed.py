import csv
import numpy as np

import matplotlib.pyplot as plt

file_path = "../resultados_87_dg.txt"
tInterval = 0.03

KGYROSPEEDCORRECT = 0.25

data = []

with open(file_path) as log:
    i = csv.reader(log)
    for line in i:
        data.append([float(line[0]), int(line[1]), int(line[2])])

last_values_odometry = [0, 0, 0, 0, 0]

last_values_gyro_1 = [2342, 2342, 2342, 2342, 2342]

last_values_gyro_2 = [2338, 2338, 2338, 2338, 2338]

data_normalized = []

data_normalized_gyro_1 = []

data_normalized_gyro_2 = []

gOffset_1 = 2342
gOffset_2 = 2338

for i in data:
    # Odometria
    last_values_odometry.pop(0)
    last_values_odometry.append(i[0])
    actual_value = sum(last_values_odometry) / len(last_values_odometry)
    actual_value_median = np.percentile(last_values_odometry, 50)
    data_normalized.append(actual_value)

    # Sensor 1
    last_values_gyro_1.pop(0)
    last_values_gyro_1.append(i[1])
    actual_value = sum(last_values_gyro_1) / len(last_values_gyro_1)

    gyroSpeed_1 = (actual_value - gOffset_1) * KGYROSPEEDCORRECT * tInterval

    gOffset_1 += (gyroSpeed_1 * KGYROSPEEDCORRECT * tInterval)

    data_normalized_gyro_1.append(gyroSpeed_1)

    # Sensor 2
    last_values_gyro_2.pop(0)
    last_values_gyro_2.append(i[2])
    actual_value = sum(last_values_gyro_2) / len(last_values_gyro_2)

    gyroSpeed_2 = (actual_value - gOffset_2) * KGYROSPEEDCORRECT * tInterval / 2

    gOffset_2 += (gyroSpeed_2 * KGYROSPEEDCORRECT * tInterval)

    data_normalized_gyro_2.append(gyroSpeed_2)


plt.plot(data_normalized, 'ro')
plt.ylabel('Odometry from wheels media')
plt.show()

plt.plot(data_normalized_gyro_1, 'ro')
plt.ylabel('Odometry from gyro 1')
plt.show()

plt.plot(data_normalized_gyro_2, 'ro')
plt.ylabel('Odometry from gyro 2')
plt.show()
