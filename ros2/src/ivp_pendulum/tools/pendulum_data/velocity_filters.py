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


position, epoch_ns, iso8601 = read_file("position_data_sets/cw_sample_set4.csv")

#  Correct single turn behaviour
position = phase_shift(position)

# plot the data
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
position = np.array(position)
epoch_ns = np.array(epoch_ns)
iso8601 = np.array(iso8601)

#ax.plot((epoch_ns - epoch_ns[0]) / 10 ** 9, position, color='r')

ax.set_title('plot')

# Finite difference method
vel = []
for i in range(1, len(position)):  # start at i=1
    dt = (epoch_ns[i] - epoch_ns[i-1])/10**9
    #print(dt)
    dx = position[i] - position[i-1]
    v = dx/dt
    vel.append(v)

time = epoch_ns[0:-1]
print(f'len vel : {len(vel)}, len time: {len(time)}')

ax.plot((time - time[0]) / 10**9, vel, color='b')


# EMA Filter - LOW-PASS: https://www.youtube.com/watch?v=1e_ZB8p5n6s
vel = []
pos_filt = [position[0]]

alpha = 0.5
for i in range(1, len(position)):
    pos_filtered = alpha * position[i] + ((1-alpha) * pos_filt[i-1])
    pos_filt.append(pos_filtered)


for i in range(1, len(pos_filt)):  # start at i=1
    dt = (epoch_ns[i] - epoch_ns[i-1])/10**9
    dx = pos_filt[i] - pos_filt[i-1]
    v = dx/dt
    vel.append(v)

time = epoch_ns[0:-1]
print(f'len vel : {len(vel)}, len time: {len(time)}')

ax.plot((time - time[0]) / 10**9, vel, color='c')

plt.show()
