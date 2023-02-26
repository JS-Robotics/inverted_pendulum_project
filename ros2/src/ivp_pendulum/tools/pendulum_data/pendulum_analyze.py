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
position1, epoch_ns1, iso86011 = read_file("position_data_sets/cw_sample_set3.csv")
position2, epoch_ns2, iso86012 = read_file("position_data_sets/cw_sample_set2.csv")
position3, epoch_ns3, iso86013 = read_file("position_data_sets/cw_sample_set5.csv")
position4, epoch_ns4, iso86014 = read_file("position_data_sets/cw_sample_set6.csv")

#  Correct single turn behaviour
position = phase_shift(position)
position1 = phase_shift(position1)
position2 = phase_shift(position2)
position3 = phase_shift(position3)
position4 = phase_shift(position4)

# plot the data
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
position = np.array(position)
epoch_ns = np.array(epoch_ns)
iso8601 = np.array(iso8601)

position1 = np.array(position1)
epoch_ns1 = np.array(epoch_ns1)
iso86011 = np.array(iso86011)

position2 = np.array(position2)
epoch_ns2 = np.array(epoch_ns2)
iso86012 = np.array(iso86012)

position3 = np.array(position3)
epoch_ns3 = np.array(epoch_ns3)
iso86013 = np.array(iso86013)

position4 = np.array(position4)
epoch_ns4 = np.array(epoch_ns4)
iso86014 = np.array(iso86014)

print(epoch_ns[0])
# ax.plot((epoch_ns - epoch_ns[0]) / 10 ** 9, position, color='r')
# ax.plot((epoch_ns1 - epoch_ns1[0]) / 10 ** 9, position1, color='b')
# ax.plot((epoch_ns2-epoch_ns2[0])/10**9, position2, color='g')
ax.plot((epoch_ns3-epoch_ns3[0])/10**9, position3, color='k')
# ax.plot((epoch_ns4-epoch_ns4[0])/10**9, position4, color='c')

# set the limits
# ax.set_xlim([0, 0.5])
# ax.set_ylim([-900, 900])

ax.set_title('plot')

# display the plot
plt.show()
