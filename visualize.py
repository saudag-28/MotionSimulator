import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# load trajectory
data = np.loadtxt("C:/Users/gsaud/OneDrive/Desktop/C++/system_design/circle.csv", delimiter=",")
x, y, theta = data[:,0], data[:,1], data[:,2]

fig, ax = plt.subplots()
ax.set_xlim(-5, 25)
ax.set_ylim(-10, 10)

# vehicle dimensions
L = 0.8  # length
W = 0.5  # width

# create rectangle
rect = patches.Rectangle((0, 0), L, W, angle=0, fc='blue')
ax.add_patch(rect)

def update(i):
    cx, cy, th = x[i], y[i], theta[i]

    # rectangle is drawn from bottom-left → shift to center
    rect.set_xy((cx - L/2, cy - W/2))
    rect.angle = np.degrees(th)

    return rect,

from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, frames=len(x), interval=50)

plt.gca().set_aspect('equal')
plt.show()