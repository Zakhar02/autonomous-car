import matplotlib.pyplot as plt
import numpy as np


def plot_car(i, w, l, xs, us, state, n, r, rad):
    plt.cla()
    i *= n
    x, y, theta = xs[i]
    v, phi = us[i]
    R = np.array([[np.cos(theta), np.sin(theta)],
                 [-np.sin(theta), np.cos(theta)]])
    car = np.array([[-l/2, -l/2, l/2, l/2, -l/2],
                   [w/2, -w/2, -w/2, w/2, w/2]])
    origin = R.T@np.array([[l/2], [0]]) + np.array([[x], [y]])
    vv = R.T@np.array([[v], [0]])
    car = R.T@car
    R = np.array([[np.cos(phi), np.sin(phi)],
                 [-np.sin(phi), np.cos(phi)]])
    vphi = R.T@vv
    plt.xlim(np.min(np.concatenate((state[:, 0], xs[:, 0]))) - 2, np.max(np.concatenate((state[:, 0], xs[:, 0]))) + 2)
    plt.ylim(np.min(np.concatenate((state[:, 1], xs[:, 1]))) - 2, np.max(np.concatenate((state[:, 1], xs[:, 1]))) + 2)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.plot(xs[:, 0], xs[:, 1], label="FMPC")
    plt.plot(state[:, 0], state[:, 1], label="Desired Trajectory")
    plt.plot(car[0, :] + x, car[1, :] + y, label="Car")
    plt.quiver(*origin, vv[0], vv[1], color="blue", label="$v$")
    plt.quiver(*origin, vphi[0], vphi[1], color="red", label="$\phi$")
    for r_, rad_ in zip(r, rad):
        circ = plt.Circle(r_, rad_, color='g')
        plt.gca().add_patch(circ)
    plt.legend()
    plt.grid()
