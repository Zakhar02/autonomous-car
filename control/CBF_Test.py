import cvxpy as cp
import numpy as np


class CBF_Test:
    def __init__(self):
        u = cp.Variable(1, name='u')
        ud = cp.Parameter(1, name="ud")
        A = cp.Parameter(name='A')
        b = cp.Parameter(name='b')
        constraints = [A*u >= b]
        cost = cp.sum_squares(u - ud)
        objective = cp.Minimize(cost/2)
        self.problem = cp.Problem(objective, constraints)

    def get_affine(self, x, xd, dxd, r, a, d):
        A = 2*np.dot((x[:-1]-r), np.array([np.cos(x[2]), np.sin(x[2])]))
        b = -a*np.sum((x[:-1]-r)**2) + a*d**2
        return A, b

    def solve(self, ud, x, r, a, d):
        self.problem.param_dict["ud"].value = ud
        A, b = self.get_affine(x, r, a, d)
        self.problem.param_dict["A"].value = A
        self.problem.param_dict["b"].value = b
        self.problem.solve()
        return self.problem.var_dict['u'].value
