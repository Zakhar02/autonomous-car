import numpy as np
from planning.DiffFlatness import DifferentialFlatness
import matplotlib.pyplot as plt
from control.MPC import FMPC


v_max = 30
phi_max = np.pi/4


def plot_state(x):
    fig, axes = plt.subplots(1, 2, sharex=True, sharey=False)
    axes[0].plot(x[0, :], label='x')
    axes[0].plot(x[1, :], label='y')
    axes[0].legend()
    axes[0].set_ylabel("Position (m)", fontsize=30)
    axes[1].plot(x[2, :], label=r"$\theta$")
    axes[1].legend()
    axes[1].set_ylabel("Angle (rad)", fontsize=30)
    axes[0].grid()
    axes[1].grid()
    fig.text(.5, .0005, 't (s)', ha='center', fontsize=30)
    plt.show()


def plot_control(u):
    fig, axes = plt.subplots(1, 2, sharex=True, sharey=False)
    axes[0].plot(u[0, :].ravel(), label='v')
    axes[0].axhline(y=v_max, label="$v_{max}$", color='b')
    axes[0].axhline(y=-v_max, label="$v_{min}$", color='b')
    axes[0].legend()
    axes[0].set_ylabel("Position (m)", fontsize=30)
    axes[1].plot(u[1, :].ravel(), label="$\phi$")
    axes[1].axhline(y=phi_max, label="$\phi_{max}$", color='b')
    axes[1].axhline(y=-phi_max, label="$\phi_{min}$", color='b')
    axes[1].legend()
    axes[1].set_ylabel("Angle (rad)", fontsize=30)
    axes[0].grid()
    axes[1].grid()
    fig.text(.5, .005, 't (s)', ha='center', fontsize=30)
    plt.show()


def plot_trajectory(state, u):
    plt.grid()
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title("Trajectory in Cartesian space", fontsize=25)
    plt.plot(state[0, :].ravel(), state[1, :].ravel())
    plt.gca().set_aspect('equal')
    plt.show()
    plot_state(state)
    plot_control(u)


def plot_flatness(state_initial, state_final, N, l, k=100):
    df = DifferentialFlatness(l)
    state, u = df.build_trajectory(state_initial, state_final, N, k)
    plot_trajectory(state, u)
    return


def plot_opti(state_initial, state_final, N, H, l, r, k=100):
    df = DifferentialFlatness(l)
    state, _ = df.build_trajectory(state_initial, state_final, N+1, k)
    tf = 20
    Q = 60*np.eye(3)
    R = np.array([[1, 0], [0, 1]])
    P = 120*np.eye(3)
    centre, rad = r
    nmpc = FMPC(N, l)
    x_ref, u_ref = nmpc.solver()(state_initial, state,
                                 tf, np.array([0, 0]), Q, R, P, v_max, phi_max, 3, 1, centre, rad)
    if rad > 0:
        plt.gca().add_patch(plt.Circle(centre, rad, color='g'))
    plot_trajectory(x_ref.full(), u_ref.full())
    return


def main():
    plt.style.use("thesis.mplstyle")
    state_initial = [0, 0, 0]
    state_final = [30, 30, np.pi+np.pi/4]
    N = 100
    l = 1
    H = 5
    # plot_flatness(state_initial, state_final, N, l)
    r = (np.array([5, 0]), 0)
    plot_opti(state_initial, state_final, N, H, l, r)


if __name__ == "__main__":
    main()
