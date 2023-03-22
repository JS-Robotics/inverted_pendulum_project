import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy.optimize import least_squares
import matplotlib.pyplot as plt


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

# Define the viscoelastic damping model
def viscoelastic_damping(params, t, y):
    beta, alpha, n = params
    v = np.gradient(y, t)
    F_damp = -beta * v - alpha * np.abs(y) ** n * np.sign(y)
    return F_damp


# Define the objective function for least-squares fitting
def objective(params, t, y):
    F_damp = viscoelastic_damping(params, t, y)
    return F_damp - y


# Load the dataset
pre_path = "../ros2/src/ivp_pendulum/tools/pendulum_data/"
position, epoch_ns, iso8601 = read_file(pre_path+"position_data_sets/cw_sample_set10.csv")
position = phase_shift(position)
shift_read_end = len(position)-10  # -2000 at 11
shift_read_start = 550 #700  # 1500
position = np.array(position[shift_read_start:shift_read_end])
epoch_ns = np.array(epoch_ns[shift_read_start:shift_read_end])
epoch_ns = (epoch_ns-epoch_ns[0])/10**9
iso8601 = np.array(iso8601[shift_read_start:shift_read_end])

t = []
y = []
n = 15
for i in range(n, len(position)-n):
    if position[i] == max(position[i-n:i+n]):
        y.append(position[i])
        t.append(epoch_ns[i])

# clean the y
remove_indexes = []
for i in range(1, len(t)-1):
    calc_period = t[i+1]-t[i]
    if calc_period <= 0.07:
        remove_indexes.append(i+1)
print(remove_indexes)
for i in reversed(remove_indexes):
    y.pop(i)
    t.pop(i)


fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(t, y, marker='o', color='b')
plt.show()

# Estimate the linear damping coefficient
slope, B = np.polyfit(t, np.log(y), 1)
beta = slope
print(slope)

# Set the initial guess for the nonlinear damping parameters
alpha_guess = 0.1 * beta
n_guess = 1.5


# Fit the viscoelastic damping model to the data
params_guess = [beta, alpha_guess, n_guess]
result = least_squares(objective, params_guess, args=(epoch_ns, position))

# Extract the nonlinear damping parameters from the result
alpha = result.x[1]
n = result.x[2]

# Print the estimated damping coefficients
print(f"Linear damping coefficient: beta = {beta}")
print(f"Nonlinear damping coefficient: alpha = {alpha}, power of displacement: n = {n}")
