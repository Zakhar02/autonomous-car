import sympy as sp
import numpy as np


class DifferentialFlatness:
    def __init__(self, l):
        self.l = l
        self.t = sp.symbols('t')
        xi = sp.symbols("xi")
        xf = sp.symbols("xf")
        alpha = sp.symbols("alpha")
        beta = sp.symbols("beta")
        x = self.trajectory()(xi, xf, alpha, beta, self.t)
        dx = x.diff(self.t)
        ddx = dx.diff(self.t)
        self.syms = [sp.lambdify([xi, xf, alpha, beta], x_)
                     for x_ in [x, dx, ddx]]

    def trajectory(self):
        return lambda xi, xf, alpha, beta, s: s**3*xf - (s - 1)**3*xi + alpha*s**2*(s - 1) + beta*s*(s - 1)**2

    def sym2f(self, sym, xi, xf, alpha, beta):
        return [f(xi, xf, alpha, beta) for f in sym]

    def build_trajectory(self, state_initial, state_final, N, k=1):
        xi, yi, thetai = state_initial
        xf, yf, thetaf = state_final
        alphax = k*np.cos(thetaf) - 3*xf
        alphay = k*np.sin(thetaf) - 3*yf
        betax = k*np.cos(thetai) + 3*xi
        betay = k*np.sin(thetai) + 3*yi
        x, dx, ddx = self.sym2f(self.syms, xi, xf, alphax, betax)
        y, dy, ddy = self.sym2f(self.syms, yi, yf, alphay, betay)
        t_ = np.linspace(0, 1, N)
        x, y, dx, dy, ddx, ddy = [np.fromiter(map(sp.lambdify(
            self.t, f), t_), dtype=np.double) for f in [x, y, dx, dy, ddx, ddy]]
        thetad = np.arctan2(dy, dx)
        vd = np.sqrt(np.square(dx)+np.square(dy))
        dthetad = np.divide(ddy*dx-ddx*dy, np.square(dx)+np.square(dy))
        phid = np.arctan2(self.l*dthetad, vd)
        return np.array([x, y, thetad]), np.array([vd, phid])
