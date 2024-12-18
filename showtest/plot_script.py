# plot_script.py
import matplotlib.pyplot as plt
import numpy as np
 
x = np.linspace(0, 2*np.pi, 400)
y = np.sin(x)
 
plt.plot(x, y)
plt.show()