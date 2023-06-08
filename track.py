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
    H = 5
    states = np.empty((1, 3))
    for landmark, k in zip(landmarks[1:], ks):
        state, _ = df.build_trajectory(state_initial, landmark, 100, k)
        states = np.vstack((states, state.T))
        state_initial = landmark
    states = states[1:]
    N = states.shape[0]
    state_f = np.tile(states[-1], (H+1, 1))
    states = np.vstack((states, state_f))
    nmpc = FMPC(H, l, 0)
    tf = 12
    dt = tf/N
    Q = 2/dt*np.eye(3)
    R = np.array([[1, 0], [0, 1/dt]])
    P = 3*np.eye(3)
    xs = np.array([0, -1, 0]).reshape(1, 3)
    us = np.array([0, 0]).reshape(1, 2)
    n = 5
    r = np.array([6, 0])
    rad = 0
    ts = []
    for i in range(N):
        time1 = time.time()
        x_ref, u_ref = df.build_trajectory(xs[-1], states[i+H+1], H+1)
        _, u = nmpc.solver()(xs[-1], x_ref,
                             dt*H, u_ref[:, :-1], Q, R, P, 30, np.pi/2.005, 3, 1, r, rad)
        ts.append(time.time()-time1)
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
        fig, plot_car, frames=N+1, fargs=(l/2, l, xs, us, states, n, r, rad))
    # animation.save('track_obstacle.gif', writer='imagemagick', fps=60)
    plt.show()
    plt.plot(ts)
    plt.show()


if __name__ == "__main__":
    main()
