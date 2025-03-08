import matplotlib.pyplot as plt
import numpy as np

voltage_input = 12
voltage_limit = 6

t = np.linspace(0, 10, 1000)

scale = 2

a = scale* np.sin(t)             + voltage_limit/2
b = scale* np.sin(t + 2*np.pi/3) + voltage_limit/2
c = scale* np.sin(t - 2*np.pi/3) + voltage_limit/2

plt.plot(t, a)
plt.plot(t, b)
plt.plot(t, c)
plt.grid(True)

plt.show()

# line to line voltages
a_b = a - b
b_c = b - c
c_a = c - a



plt.plot(t, a_b)
plt.plot(t, b_c)
plt.plot(t, c_a)
plt.plot(t, current_vector)

plt.grid(True)
plt.show()