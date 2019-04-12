import collections
import csv
import time

import numpy as np

import matplotlib.pyplot as plt

file_path = "../resultados_250_dg.txt"
d_t = 0.03

KGYROSPEEDCORRECT = 0.25

data = []

with open(file_path) as log:
    i = csv.reader(log)
    for line in i:
        data.append([float(line[0]), int(line[1]), int(line[2])])

t_next_period = time.time()

final_data = []

# Gyro sensor offset and calibration
gyro_1_offset = 2342
gyro_2_offset = 2338

# last values that the odometry got
last_values_w_odo = collections.deque(5 * [0], 5)

last_values_gyro_1 = collections.deque(5 * [gyro_1_offset], 5)

last_values_gyro_2 = collections.deque(5 * [gyro_2_offset], 5)

w_odo_accumulated = 0
gyro_1_accumulated = 0
gyro_2_accumulated = 0

gyro_1_correction_factor = 0.17
gyro_2_correction_factor = 0.11

gyro_1_offset_correction_factor = 0
gyro_2_offset_correction_factor = 0

angle = 0

for i in data:
    actual_value_od = i[0]
    w_sensor = i[1]
    w_sensor_2 = i[2]

    # Odometria
    gyro_1_accumulated += w_sensor - last_values_gyro_1.popleft()
    last_values_gyro_1.append(w_sensor)
    actual_value_gyro_1 = gyro_1_accumulated / 5
    #actual_value_gyro_1 = sum(last_values_gyro_1) / 5

    actual_value_gyro_1 = - (actual_value_gyro_1 - gyro_1_offset) * gyro_1_correction_factor * d_t

    gyro_1_offset += (actual_value_gyro_1 * gyro_1_offset_correction_factor * d_t)

    # Sensor 2
    last_values_gyro_2.append(w_sensor_2)
    actual_value_gyro_2 = sum(last_values_gyro_2) / len(last_values_gyro_2)

    actual_value_gyro_2 = - (actual_value_gyro_2 - gyro_2_offset) * gyro_2_correction_factor * d_t

    gyro_2_offset += (actual_value_gyro_2 * gyro_2_offset_correction_factor * d_t)

    w = (actual_value_od + actual_value_gyro_1 + actual_value_gyro_2) / 3.0
    final_data.append(w)

    angle += w * d_t

t_next_period_2 = time.time()

# plt.plot(final_data, 'ro')
# plt.ylabel('Odometry th')
# plt.show()

print (np.rad2deg(angle))
