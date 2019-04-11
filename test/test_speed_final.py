import csv
import numpy as np

import matplotlib.pyplot as plt

file_path = "../resultados_250_dg.txt"
tInterval = 0.03

KGYROSPEEDCORRECT = 0.25

data = []

with open(file_path) as log:
    i = csv.reader(log)
    for line in i:
        data.append([float(line[0]), int(line[1]), int(line[2])])

final_data = []

last_values_odometry = [0, 0, 0, 0, 0]

last_values_gyro_1 = [2342, 2342, 2342, 2342, 2342]

last_values_gyro_2 = [2338, 2338, 2338, 2338, 2338]


gOffset_1 = 2342
gOffset_2 = 2338

correct_gyro_1 = 0.17
correct_gyro_2 = 0.11

angle = 0

for i in data:
    # Odometria
    last_values_odometry.pop(0)
    last_values_odometry.append(i[0])
    actual_value_od = sum(last_values_odometry) / len(last_values_odometry)

    # Sensor 1
    last_values_gyro_1.pop(0)
    last_values_gyro_1.append(i[1])
    actual_value_gyro_1 = sum(last_values_gyro_1) / len(last_values_gyro_1)

    actual_value_gyro_1 = (actual_value_gyro_1 - gOffset_1) * correct_gyro_1 * tInterval

    gOffset_1 += (actual_value_gyro_1 * KGYROSPEEDCORRECT * tInterval)

    actual_value_gyro_1 = - actual_value_gyro_1

    # Sensor 2
    last_values_gyro_2.pop(0)
    last_values_gyro_2.append(i[2])
    actual_value_gyro_2 = sum(last_values_gyro_2) / len(last_values_gyro_2)

    actual_value_gyro_2 = (actual_value_gyro_2 - gOffset_2) * correct_gyro_2 * tInterval

    gOffset_2 += (actual_value_gyro_2 * KGYROSPEEDCORRECT * tInterval)

    actual_value_gyro_2 = - actual_value_gyro_2

    media_total = (actual_value_od + actual_value_gyro_1 + actual_value_gyro_2) / 3
    final_data.append(media_total)

    angle += media_total * tInterval


plt.plot(final_data, 'ro')
plt.ylabel('Odometry from wheels media')
plt.show()

print (np.rad2deg(angle))
