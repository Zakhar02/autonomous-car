import numpy as np
import matplotlib.pyplot as plt
from planning.DiffFlatness import DifferentialFlatness


def main():
    landmarks = np.array([[0, 0, 0], [4, 4, np.pi/4], [5, 5, 0], [9, 7, np.pi/2], [0, 7, np.pi+np.pi/4], [0, 0, 0]])
    df = DifferentialFlatness(1)
    state_initial = landmarks[0]
    H = 10
    states = np.empty((1, 3))
    for landmark in landmarks[1:]:
        state, _ = df.build_trajectory(state_initial, landmark, H, 15)
        states = np.vstack((states, state.T))
        state_initial = landmark
    states = states[1:]
    plt.plot(states[:, 0], states[:, 1])
    plt.show()


if __name__ == "__main__":
    main()
