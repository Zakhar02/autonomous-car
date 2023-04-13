from control.MPC import FMPC
from planning.DiffFlatness import DifferentialFlatness
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import time


def plot_car(i, w, l, xs, us, state):
    plt.cla()
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
    plt.xlabel('x')
    plt.ylabel('y')
    plt.plot(xs[:, 0], xs[:, 1], label="NMPC")
    plt.plot(state[0, :], state[1, :], label="Flat Output")
    plt.plot(car[0, :] + x, car[1, :] + y, label="Car")
    plt.quiver(*origin, vv[0], vv[1], color="blue", label="$v$")
    plt.quiver(*origin, vphi[0], vphi[1], color="red", label="$\phi$")
    plt.legend()


def main():
    H = 5
    N = 100
    l = 1
    state_initial = [0, 0, 0]
    state_final = [30, 30, np.pi/2]
    df = DifferentialFlatness(N+1, l)
    state, _ = df.build_trajectory(state_initial, state_final, 100)
    state_f = np.tile(state[:, -1], (H+1, 1)).T
    state = np.vstack((state.T, state_f.T)).T
    Q = 15*np.eye(3)
    R = np.array([[1, 0], [0, 10]])
    P = 100*np.eye(3)
    xs = np.array(state_initial).reshape(1, 3)
    us = np.array([0, 0]).reshape(1, 2)
    nmpc = FMPC(H, l)
    tf = 10
    dt = tf/N
    time1 = time.time()
    for i in range(N):
        _, u = nmpc.solver()(xs[-1], state[:, i:i+H+1],
                             tf*H/N, np.zeros(2), Q, R, P, 30, np.pi/4, 3, 1)
        us = np.vstack((us, u[:, 0].T))
        x_next = nmpc.rk4(xs[-1], us[-1], dt)
        xs = np.vstack((xs, x_next.T))
    time1 -= time.time()
    print(f"MPC calculation time is {-time1} seconds")
    fig = plt.figure()
    animation = FuncAnimation(
        fig, plot_car, frames=N+1, fargs=(l/2, l, xs, us, state))
    animation.save('mpc.gif', writer='imagemagick', fps=60)
    # plt.show()


if __name__ == "__main__":
    main()
