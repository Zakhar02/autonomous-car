import sympy as sp
import numpy as np


class DifferentialFlatness:
    def __init__(self, N, l):
        self.l = l
        self.N = N

    def trajectory(self, xi, xf, alpha, beta):
        return lambda s: s**3*xf - (s - 1)**3*xi + alpha*s**2*(s - 1) + beta*s*(s - 1)**2

    def build_trajectory(self, state_initial, state_final, k=1):
        xi, yi, thetai = state_initial
        xf, yf, thetaf = state_final
        alphax = k*np.cos(thetaf) - 3*xf
        alphay = k*np.sin(thetaf) - 3*yf
        betax = k*np.cos(thetai) + 3*xi
        betay = k*np.sin(thetai) + 3*yi
        t_ = np.linspace(0, 1, self.N)
        x = self.trajectory(xi, xf, alphax, betax)
        y = self.trajectory(yi, yf, alphay, betay)
        t = sp.symbols('t')
        dx = x(t).diff(t)
        dy = y(t).diff(t)
        ddx = dx.diff(t)
        ddy = dy.diff(t)
        x, y, dx, dy, ddx, ddy = [np.fromiter(map(sp.lambdify(
            t, f), t_), dtype=np.double) for f in [x(t), y(t), dx, dy, ddx, ddy]]
        thetad = np.arctan2(dy, dx)
        vd = np.sqrt(np.square(dx)+np.square(dy))
        dthetad = np.divide(ddy*dx-ddx*dy, np.square(dx)+np.square(dy))
        phid = np.arctan2(self.l*dthetad, vd)
        return np.array([x, y, thetad]), np.array([vd, phid])
