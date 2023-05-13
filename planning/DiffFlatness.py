import sympy as sp
import numpy as np


class DifferentialFlatness:
    def __init__(self, l):
        self.l = l
        t = sp.symbols('t')
        xi = sp.symbols("xi")
        xf = sp.symbols("xf")
        alpha = sp.symbols("alpha")
        beta = sp.symbols("beta")
        phi_s = self.trajectory()(xi, xf, alpha, beta, t)
        dphi_s = phi_s.diff(t)
        ddphi_s = dphi_s.diff(t)
        self.syms = [sp.lambdify([xi, xf, alpha, beta, t], func_)
                     for func_ in [phi_s, dphi_s, ddphi_s]]
        self.phi = self.syms[0]
        self.dphi = self.syms[1]
        self.ddphi = self.syms[2]

    def trajectory(self):
        return lambda xi, xf, alpha, beta, s: s**3*xf - (s - 1)**3*xi + alpha*s**2*(s - 1) + beta*s*(s - 1)**2

    def build_trajectory(self, state_initial, state_final, N, k=1):
        xi, yi, thetai = state_initial
        xf, yf, thetaf = state_final
        alphax = k*np.cos(thetaf) - 3*xf
        alphay = k*np.sin(thetaf) - 3*yf
        betax = k*np.cos(thetai) + 3*xi
        betay = k*np.sin(thetai) + 3*yi
        t = np.linspace(0, 1, N)
        x = self.phi(xi, xf, alphax, betax, t)
        y = self.phi(yi, yf, alphay, betay, t)
        dx = self.dphi(xi, xf, alphax, betax, t)
        dy = self.dphi(yi, yf, alphay, betay, t)
        ddx = self.ddphi(xi, xf, alphax, betax, t)
        ddy = self.ddphi(yi, yf, alphay, betay, t)
        thetad = np.arctan2(dy, dx)
        vd = np.sqrt(np.square(dx)+np.square(dy))
        dthetad = np.divide(ddy*dx-ddx*dy, np.square(dx)+np.square(dy))
        phid = np.arctan2(self.l*dthetad, vd)
        return np.array([x, y, thetad]), np.array([vd, phid])
