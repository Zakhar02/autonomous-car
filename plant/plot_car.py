import matplotlib.pyplot as plt
import numpy as np


def plot_car(i, w, l, xs, us, state, n):
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
    plt.xlim(np.min(state[0, :]) - 2, np.max(state[0, :]) + 2)
    plt.ylim(np.min(state[1, :]) - 2, np.max(state[1, :]) + 2)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.plot(xs[:, 0], xs[:, 1], label="FMPC")
    plt.plot(state[0, :], state[1, :], label="Flat Output")
    plt.plot(car[0, :] + x, car[1, :] + y, label="Car")
    plt.quiver(*origin, vv[0], vv[1], color="blue", label="$v$")
    plt.quiver(*origin, vphi[0], vphi[1], color="red", label="$\phi$")
    plt.legend()
    plt.grid()
