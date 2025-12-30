from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np


# Pseudo data
x = np.linspace(0, 10, 11)
y = np.cos(-x**2/9.0)

# Cubic spline interpolation
x_fine = np.linspace(0, 10, 1000)
f_interpolate = interp1d(x, y, kind='cubic')
y_fine = f_interpolate(x_fine)

print(len(y), type(y))
print(len(y_fine), type(y_fine))

'''
# Plot
plt.plot(x, y, 'b-', alpha=0.8, label='Linear')
plt.plot(x_fine, y_fine, 'r-', alpha=0.8, label='Cubic')
plt.plot(x, y, 'ko', alpha=0.8, label='Data')
plt.legend()
plt.show()
'''
