import csv
import matplotlib.pyplot as plt

file_path = "../resultados_giro_87_dg.txt"
period = 0.015

data = []

with open(file_path) as log:
    i = csv.reader(log)
    for line in i:
        data.append([float(line[0]), int(line[1])])

plt.plot(list(map(lambda a: a[0], data)), 'ro')
plt.ylabel('Odometry from wheels')
plt.show()

plt.plot(list(map(lambda a: a[1], data)), 'ro')
plt.ylabel('Odometry from gyro')
plt.show()
