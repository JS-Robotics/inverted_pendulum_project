#
#  Created by ICraveSleep on 23.02.23.
#

import csv
import matplotlib.pyplot as plt
import numpy as np


def read_file(file_name):
    with open(file_name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        position_ = []
        epoch_ns_ = []
        iso8601_ = []
        for row in csv_reader:
            if line_count == 0:
                print(row)
            elif line_count == 1:
                print(row)
            else:
                position_.append(float(row[0]))
                epoch_ns_.append(float(row[1]))
                iso8601_.append(str(row[2]))
            line_count += 1
        return position_, epoch_ns_, iso8601_


def phase_shift(pos_set):
    max_14bit = 16383
    for i in range(len(pos_set)):
        pos = pos_set[i]
        if pos > 9000:
            pos_set[i] = pos - max_14bit
    return pos_set


pre_path = "../ros2/src/ivp_pendulum/tools/pendulum_data/"

position, epoch_ns, iso8601 = read_file(pre_path+"position_data_sets/cw_sample_set10.csv")

#  Correct single turn behaviour
position = phase_shift(position)

# plot the data
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
shift_read_end = len(position)-800  # -2000 at 11
shift_read_start = 550 #700  # 1500
print(shift_read_end)
position = np.array(position[shift_read_start:shift_read_end])
position = position*2*np.pi/16382  # Convert to rad
epoch_ns = np.array(epoch_ns[shift_read_start:shift_read_end])
iso8601 = np.array(iso8601[shift_read_start:shift_read_end])


step = 0
pos = position[step]
pos_old = pos - 100
points = []
points_time = []

n = 15
for i in range(n, len(position)-n):
    if position[i] == max(position[i-n:i+n]):
        points.append(position[i])
        points_time.append(((epoch_ns[i])-epoch_ns[0])/10**9)


# clean the points
remove_indexes = []
for i in range(1, len(points_time)-1):
    # print(f"Period at {i}: {points_time[i+1]-points_time[i]}")
    calc_period = points_time[i+1]-points_time[i]
    if calc_period <= 0.07:
        remove_indexes.append(i+1)
print(remove_indexes)
for i in reversed(remove_indexes):
    points.pop(i)
    points_time.pop(i)

# Poly fit
points = np.array(points)
points_time = np.array(points_time)

A, B = np.polyfit(points_time, np.log(points), 1)


######  RUN EVERYTHING OVER AGAIN WITH ALL SAMPLES

position, epoch_ns, iso8601 = read_file(pre_path+"position_data_sets/cw_sample_set10.csv")
#  Correct single turn behaviour
position = phase_shift(position)

shift_read_end = len(position)-0
shift_read_start = 0
position = np.array(position[shift_read_start:shift_read_end])
position = position*2*np.pi/16382  # Convert to rad
epoch_ns = np.array(epoch_ns[shift_read_start:shift_read_end])
iso8601 = np.array(iso8601[shift_read_start:shift_read_end])


step = 0
pos = position[step]
pos_old = pos - 100
points = []
points_time = []

n = 15
for i in range(n, len(position)-n):
    if position[i] == max(position[i-n:i+n]):
        points.append(position[i])
        points_time.append(((epoch_ns[i])-epoch_ns[0])/10**9)


# clean the points
remove_indexes = []
for i in range(1, len(points_time)-1):
    # print(f"Period at {i}: {points_time[i+1]-points_time[i]}")
    calc_period = points_time[i+1]-points_time[i]
    if calc_period <= 0.07:
        remove_indexes.append(i+1)
print(remove_indexes)
for i in reversed(remove_indexes):
    points.pop(i)
    points_time.pop(i)

points = np.array(points)
points_time = np.array(points_time)

sample_set = ax.plot(points_time, np.log(points), 'ko', label='Sample set')
linear_fit = ax.plot(points_time, A*points_time+B+0.33, color='r', label="Linear fit")

ax.set_title('Log of Sample data')
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Amplitude [-]")
plt.show()
