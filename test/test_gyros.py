import csv

file_path = "../la_mejor_pipe.txt"

w_values = []

with open(file_path) as log:
    i = csv.reader(log)
    for line in i:
        w_wheels = float(line[0])
        w_gyro_1 = float(line[1])
        w_gyro_2 = float(line[2])
        raw_gyro_1 = float(line[3])
        raw_gyro_2 = float(line[4])
        offset_gyro_1 = float(line[5])
        offset_gyro_2 = float(line[6])
        w_values.append([w_wheels, w_gyro_1, w_gyro_2, raw_gyro_1, raw_gyro_2, offset_gyro_1, offset_gyro_2])

for i in w_values:
    w_wheels, w_gyro_1, w_gyro_2, raw_gyro_1, raw_gyro_2, offset_gyro_1, offset_gyro_2 = i

    print(w_wheels, w_gyro_1, w_gyro_2, raw_gyro_1, raw_gyro_2, offset_gyro_1, offset_gyro_2)
