import cvxpy as cp
import numpy as np


class CBF:
    def __init__(self):
        u = cp.Variable(2, name='u')
        ud = cp.Parameter(2, name="ud")
        A = cp.Parameter(2, name='A')
        b = cp.Parameter(name='b')
        constraints = [A@u >= b]
        cost = cp.sum_squares(u - ud)
        objective = cp.Minimize(cost/2)
        self.problem = cp.Problem(objective, constraints)

    def get_affine(self, x, xd, dxd, d, a):
        e = xd - x
        p = np.array([-np.sin(x[2]), np.cos(x[2]), 0])
        P = np.array([[0, -np.cos(x[2])],
                     [0, -np.sin(x[2])], [0, 0]])
        A = 2*np.dot(e, p)*e.T@P
        b = np.dot(e, p)*(a*np.dot(e, p) - 2*np.dot(dxd, p)) + a*d**2
        return A, b

    def solve(self, ud, x, xd, dxd, d, a):
        self.problem.param_dict["ud"].value = ud
        A, b = self.get_affine(x, xd, dxd, d, a)
        self.problem.param_dict["A"].value = A
        self.problem.param_dict["b"].value = b
        self.problem.solve()
        return self.problem.var_dict['u'].value
