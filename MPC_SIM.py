from control.MPC import FMPC
from planning.DiffFlatness import DifferentialFlatness
import matplotlib.pyplot as plt
import numpy as np
import time


if __name__ == "__main__":
    H = 10
    N = 100
    state_initial = [0, 0, 0]
    state_final = [30, 30, np.pi/2]
    df = DifferentialFlatness(N+1, 1)
    state, _ = df.build_trajectory(state_initial, state_final)
    state_f = np.tile(state[:, -1], (H+1, 1)).T
    state = np.vstack((state.T, state_f.T)).T
    Q = 10*np.eye(3)
    R = 1*np.array([[1, 0], [0, 1]])
    P = 200*np.eye(3)
    xs = np.array(state_initial).reshape(1, 3)
    u0 = [0, 0]
    nmpc = FMPC(H, 1)
    tf = 10
    dt = tf/N
    time1 = time.time()
    for i in range(N):
        x, u = nmpc.solver()(xs[-1], state[:, i:i+H+1],
                             tf*H/N, np.zeros(2), Q, R, P, 30, np.pi/4, 3, 1)
        u0 = u[:, 0]
        x_next = nmpc.rk4(xs[-1], u0, dt)
        xs = np.vstack((xs, x_next.T))
    time1 -= time.time()
    print(f"MPC calculation time is {-time1} seconds")
    plt.plot(xs[:, 0], xs[:, 1], label="NMPC")
    plt.plot(state[0, :], state[1, :], label="Flat Output")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.show()
