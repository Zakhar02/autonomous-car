from casadi import *


class FMPC:
    def __init__(self, N, l):
        opti = Opti()
        tf = opti.parameter()
        dt = tf/N
        x = opti.variable(3, N+1)
        u = opti.variable(2, N)
        x_ref = opti.parameter(3, N+1)
        u_ref = opti.parameter(2, N)
        x0 = opti.parameter(3)
        Q = opti.parameter(3, 3)
        R = opti.parameter(2, 2)
        P = opti.parameter(3, 3)
        v_max = opti.parameter()
        phi_max = opti.parameter()
        acc_max = opti.parameter()
        dphi_max = opti.parameter()
        r = opti.parameter(1, 2)
        d = opti.parameter()
        self.f = Function('f', [x, u], [vertcat(
            u[0] * cos(x[2]), u[0] * sin(x[2]), (u[0]/l) * tan(u[1]))])
        cost = (x[:, -1] - x_ref[:, -1]).T@P@(x[:, -1] - x_ref[:, -1])
        opti.subject_to((x[:2, -1] - r.T).T@(x[:2, -1] - r.T) > d**2)
        for i in range(N):
            opti.subject_to(x[:, i+1] == self.rk4(x[:, i], u[:, i], dt))
            cost += (x[:, i] - x_ref[:, i]).T@Q@(x[:, i] - x_ref[:, i]) * \
                dt + (u[:, i] - u_ref[:, i]).T@R@(u[:, i] - u_ref[:, i])*dt
            opti.subject_to(opti.bounded(-phi_max, u[1, i], phi_max))
            opti.subject_to(opti.bounded(-v_max, u[0, i], v_max))
            opti.subject_to((x[:2, i] - r.T).T@(x[:2, i] - r.T) > d**2)
            if i == 0:
                continue
            opti.subject_to(
                opti.bounded(-acc_max*dt, u[0, i] - u[0, i-1], acc_max*dt))
            opti.subject_to(opti.bounded(-dphi_max*dt,
                            u[1, i] - u[1, i-1], dphi_max*dt))
        opti.minimize(cost)
        opti.subject_to(x[:, 0] == x0)
        options = {
            "print_level": 0
        }
        coptions = {
            'compiler': 'shell',
            'jit': True,
            'jit_options': {'compiler': 'gcc'},
            'print_time': False,
            "expand": True,
        }
        opti.solver('ipopt', coptions, options)
        self.nmpc = opti.to_function(
            "nmpc", [x0, x_ref, tf, u_ref, Q, R, P, v_max, phi_max, acc_max, dphi_max, r, d], [x, u])

    def rk4(self, x, u, dt):
        k1 = self.f(x, u)
        k2 = self.f(x + dt/2 * k1, u)
        k3 = self.f(x + dt/2 * k2, u)
        k4 = self.f(x + dt * k3, u)
        return x + dt/6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def solver(self):
        return self.nmpc
