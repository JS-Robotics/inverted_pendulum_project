import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def rayleigh_damping(v, a, b):
    return a * v + b * v**2

# Simulate a dataset with non-linear damping
t = np.linspace(0, 90, 1000)
omega = np.sqrt(9.81)
theta = 0.1 * np.exp(-0.2*t) * np.sin(omega*t)
v = np.diff(theta) / np.diff(t)
a = np.diff(v) / np.diff(t)
d = 0.05 * v + 0.01 * v**2 + 0.002 * a**2

# Fit a Rayleigh damping function to the data
popt, pcov = curve_fit(rayleigh_damping, v, d)

# Plot the data and the fitted function
plt.plot(v, d, '.', label='data')
v_fit = np.linspace(np.min(v), np.max(v), 100)
d_fit = rayleigh_damping(v_fit, *popt)
plt.plot(v_fit, d_fit, '-', label='fit')
plt.xlabel('velocity (rad/s)')
plt.ylabel('damping coefficient (s^-1)')
plt.legend()
plt.show()

# Print the fitted parameters
print("Fitted parameters: a={:.4f}, b={:.4f}".format(*popt))