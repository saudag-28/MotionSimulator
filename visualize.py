import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.transforms as transforms


# ----------------------------
# CONFIG
# ----------------------------
MODE = "multi"   # "single" or "multi"

L, W = 0.8, 0.5


# ----------------------------
# SINGLE AGENT VISUALIZATION
# ----------------------------
def run_single():
    data = np.loadtxt("res/circle_.csv", delimiter=",")

    x, y, theta = data[:, 0], data[:, 1], data[:, 2]

    fig, ax = plt.subplots()
    ax.set_xlim(-5, 25)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')

    rect = patches.Rectangle((-L/2, -W/2), L, W, fc='blue')
    ax.add_patch(rect)

    def update(i):
        cx, cy, th = x[i], y[i], theta[i]

        t = (transforms.Affine2D()
            .rotate(th)
            .translate(cx, cy))

        rect.set_transform(t + ax.transData)

        # direction vector (tangent)
        dx = np.cos(th)
        dy = np.sin(th)

        return rect

    ani = FuncAnimation(fig, update, frames=len(x), interval=50, repeat=True)
    
    # plt.plot(theta)
    plt.show()

    # plt.gca().set_aspect('equal')
    # plt.show()

# ----------------------------
# MULTI AGENT + COLLISION
# ----------------------------
def run_multi():
    data = np.loadtxt("res/traj_multi.csv", delimiter=",")

    x1, y1, th1 = data[:, 0], data[:, 1], data[:, 2]
    x2, y2, th2 = data[:, 3], data[:, 4], data[:, 5]

    # collision info (optional)
    try:
        cdata = np.loadtxt("res/collision.txt", delimiter=",")
        collision_step = int(cdata[0])
        cx, cy = cdata[1], cdata[2]
    except:
        collision_step = -1
        cx, cy = None, None

    fig, ax = plt.subplots()
    ax.set_xlim(-5, 25)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')

    rect1 = patches.Rectangle((-L/2, -W/2), L, W, fc='blue')
    rect2 = patches.Rectangle((-L/2, -W/2), L, W, fc='green')

    ax.add_patch(rect1)
    ax.add_patch(rect2)

    collision_marker, = ax.plot([], [], 'r*', markersize=15)

    def update(i):
        # agent 1
        t1 = (transforms.Affine2D()
            .rotate(th1[i])
            .translate(x1[i], y1[i]))

        rect1.set_transform(t1 + ax.transData)

        # agent 2
        t2 = (transforms.Affine2D()
            .rotate(th2[i])
            .translate(x2[i], y2[i]))

        rect2.set_transform(t2 + ax.transData)

        # collision display
        if i == collision_step:
            collision_marker.set_data([cx], [cy])
        else:
            collision_marker.set_data([], [])

        return rect1, rect2, collision_marker

    ani = FuncAnimation(fig, update, frames=len(x1), interval=50)
    ani.save("res/TwoAgents.gif", writer="pillow", fps=20)
    plt.show()



if __name__ == "__main__":
    if MODE == "single":
        run_single()
    elif MODE == "multi":
        run_multi()
    else:
        raise ValueError("MODE must be 'single' or 'multi'")