import numpy as np
import matplotlib.pyplot as plt
from planning.DiffFlatness import DifferentialFlatness
from control.MPC import FMPC
import time
from scipy.interpolate import CubicSpline
from matplotlib.animation import FuncAnimation
from plant.plot_car import plot_car


def main():
    l = 1
    landmarks = np.array([[0, 0, 0], [5, 2, np.pi/3], [6, 6, 3*np.pi/4], [0, 6, -3*np.pi/4], [0, 3, -np.pi/3],
                         [12, 7, np.pi/2], [0, 9, np.pi+np.pi/4], [-5, 8, 5*np.pi/4], [0, 0, 0]])
    ks = [5, 5, 5, 5, 20, 10, 2, 23]
    df = DifferentialFlatness(l)
    state_initial = landmarks[0]
    H = 3
    states = np.empty((1, 3))
    u_ref = np.empty((1, 2))
    for landmark, k in zip(landmarks[1:], ks):
        state, u = df.build_trajectory(state_initial, landmark, 100, k)
        states = np.vstack((states, state.T))
        state_initial = landmark
        u_ref = np.vstack((u_ref, u.T))
    states = states[1:]
    N = states.shape[0]
    state_f = np.tile(states[-1], (H+1, 1))
    u_f = np.tile(u_ref[-1], (H, 1))
    states = np.vstack((states, state_f))
    u_ref = np.vstack((u_ref, u_f))
    nmpc = FMPC(H, l)
    Q = 40*np.eye(3)
    R = np.array([[1, 0], [0, 1]])
    P = 110*np.eye(3)
    xs = np.array([0, -1, 0]).reshape(1, 3)
    us = np.array([0, 0]).reshape(1, 2)
    tf = 7
    dt = tf/N
    n = 5
    time1 = time.time()
    for i in range(N):
        _, u = nmpc.solver()(xs[-1], states[i:i+H+1].T,
                             dt*H, u_ref[i:i+H].T, Q, R, P, 30, np.pi/2.005, 3, 1)
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
        fig, plot_car, frames=N+1, fargs=(l/2, l, xs, us, states, n))
    animation.save('track.gif', writer='imagemagick', fps=60)
    # plt.show()


if __name__ == "__main__":
    main()
