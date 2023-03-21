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
epoch_ns = np.array(epoch_ns[shift_read_start:shift_read_end])
iso8601 = np.array(iso8601[shift_read_start:shift_read_end])


step = 0
pos = position[step]
pos_old = pos - 100
points = []
points_time = []

# for i in range(49):
#     while pos >= pos_old:
#         pos_old = pos
#         step += 1
#         pos = position[step]
#
#     points.append(position[step - 1])
#     points_time.append((epoch_ns[step - 1] - epoch_ns[0]) / 10 ** 9)
#
#     while pos <= pos_old:
#         pos_old = pos
#         step += 1
#         pos = position[step]

    #points.append(position[step - 1])
    #points_time.append((epoch_ns[step - 1] - epoch_ns[0]) / 10 ** 9)
n = 15
for i in range(n, len(position)-n):
    if position[i] == max(position[i-n:i+n]):
        points.append(position[i])
        points_time.append(((epoch_ns[i])-epoch_ns[0])/10**9)


# clean the points
remove_indexes = []
for i in range(1, len(points_time)-1):
    print(f"Period at {i}: {points_time[i+1]-points_time[i]}")
    calc_period = points_time[i+1]-points_time[i]
    if calc_period <= 0.07:
        remove_indexes.append(i+1)
print(remove_indexes)
for i in reversed(remove_indexes):
    points.pop(i)
    points_time.pop(i)


n_c = len(points)-1
d = np.log(points[0] / points[-1])
print(len(points))
print(points)
c = d/(np.sqrt(d**2 + (n_c*2*np.pi)**2))
dt = 2
t = np.zeros(45)

timer = 0
for step in range(len(t)):
    t[step] = timer
    timer += dt




# Poly fit
points = np.array(points)
points_time = np.array(points_time)
A, B = np.polyfit(points_time, np.log(points), 1)

c1 = 0.050
c2 = 0.0001

c3 = 0.075
c4 = 0.0325
c5 = 0.000001
c6 = 0.04315
e = (points[0])*np.exp(-c1*t)
e_l = (points[0])*np.exp(A*t)
e_2 = points[0]*np.exp(-c6*t)

t = t + points_time[0]




T = np.median(points_time[1:] - points_time[0:np.size(points)-1])
print(T)
print(points_time[1:] - points_time[0:np.size(points)-1])
print(points_time[5] - points_time[4])
t_new = np.zeros(1000)
dt = 0.08
timer = 0

for step in range(len(t_new)):
    t_new[step] = timer
    timer += dt
plotter = np.exp(A*t_new + B) * np.sin(2*np.pi/T*t_new + 0.95)
e_new = (A*t_new + np.exp(B)) * np.sin(2*np.pi/T*t_new + 0.25)

ax.plot((epoch_ns - epoch_ns[0]) / 10 ** 9, position, color='r')
ax.plot(points_time, points, marker='o', color='b')
#ax.plot(t, e, color='c')
ax.plot(t, e_l, color='g')
ax.plot(t, e_2, color='k')
ax.plot(t_new, plotter, color='y')
# ax.plot(t_new, e_new, color='k')
# ax.plot((epoch_ns1 - epoch_ns1[0]) / 10 ** 9, position1, color='b')
# ax.plot((epoch_ns2-epoch_ns2[0])/10**9, position2, color='g')
# ax.plot((epoch_ns3-epoch_ns3[0])/10**9, position3, color='k')
# ax.plot((epoch_ns4-epoch_ns4[0])/10**9, position4, color='c')


ax.set_title('plot')

# display the plot

plt.figure()
plt.plot(points_time, np.log(points), 'ks')
plt.plot(points_time, A*points_time + B, 'r')
plt.show()

# https://www.youtube.com/watch?v=z3jaxxK1Pt8