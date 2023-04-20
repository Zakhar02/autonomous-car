import cvxpy as cp
import numpy as np


class CBF:
    def __init__(self, H, l):
        u = cp.Variable((2, H), name='u')
        ud = cp.Parameter((2, H), name="ud")
        x = cp.Parameter((3, H+1), name='x')
        xd = cp.Parameter((3, H+1), name="xd")
        dxd = cp.Parameter((3, H+1), name="dxd")
        tf = cp.Parameter(1, name="tf")
        d = cp.Parameter(1, name='d')
        a = cp.Parameter(1, name='a')
        cost = 0
        constraints = []
        for i in range(H):
            cost += cp.sum_squares(u[:, i] - ud[:, i])
            e = xd[:, i] - x[:, i]
            p = np.array([[-np.sin(x[2, i]), np.cos(x[2, i]), 0]]).T
            P = np.array([[0, -np.cos(x[2, i])], [0, -np.sin(x[2, i])], [0, 0]])
            constraints.append(2*e@p@e.T@P@u[:, i] >= e@p@(a*e@p - 2*dxd[:, i]@p) + a*cp.square(d))
        objective = cp.Minimize(cost/2)
        self.problem = cp.Problem(objective, constraints)
    
    def solve(self, ud, x, xd, dxd, tf, d, a):
        self.problem.param_dict["ud"].value = ud
        self.problem.param_dict["x"].value = x
        self.problem.param_dict["xd"].value = xd
        self.problem.param_dict["dxd"].value = dxd
        self.problem.param_dict["tf"].value = tf
        self.problem.param_dict["d"].value = d
        self.problem.param_dict["a"].value = a
        return self.problem.solve()

