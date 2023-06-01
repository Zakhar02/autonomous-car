from control.MPC import FMPC
from planning.DiffFlatness import DifferentialFlatness
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time
from scipy.interpolate import CubicSpline
from plant.plot_car import plot_car


def main():
    H = 5
    N = 100
    l = 1
    state_initial = [0, 0, 0]
    state_final = [30, 30, 0]
    df = DifferentialFlatness(l)
    state, _ = df.build_trajectory(state_initial, state_final, N, 100)
    state_f = np.tile(state[:, -1], (H+1, 1)).T
    state = np.vstack((state.T, state_f.T)).T
    xs = np.array([1, 1, 0]).reshape(1, 3)
    us = np.array([0, 0]).reshape(1, 2)
    tf = 10
    dt = tf/N
    n = 5
    r = np.array([5, 0])
    rad = 2
    time1 = time.time()
    nmpc = FMPC(H, l)
    Q = 220*np.eye(3)
    R = np.array([[1, 0], [0, 1]])
    P = 180*np.eye(3)
    for i in range(N):
        x_ref, u_ref = df.build_trajectory(xs[-1], state[:, i+H+1], H+1)
        _, u = nmpc.solver()(xs[-1], x_ref,
                             dt*H, u_ref[:, :-1], Q, R, P, 30, np.pi/4, 3, 1, r, rad)
        vi, phii = [CubicSpline(np.linspace(0, dt*H, H), u.full()[i, :].ravel())
                    for i in range(u.shape[0])]
        for u_ in zip(vi(np.linspace(0, dt, n)), phii(np.linspace(0, dt, n))):
            us = np.vstack((us, u_))
            x_next = nmpc.rk4(xs[-1], us[-1], dt/n)
            xs = np.vstack((xs, x_next.T))
    time1 -= time.time()
    print(f"MPC calculation time is {-time1} seconds")
    fig = plt.figure()
    animation = FuncAnimation(
        fig, plot_car, frames=N+1, fargs=(l/2, l, xs, us, state, n, r, rad))
    animation.save('mpc.gif', writer='imagemagick', fps=60)
    # plt.show()


if __name__ == "__main__":
    main()
