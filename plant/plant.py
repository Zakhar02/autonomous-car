import lcm
from car import init_state, control_cmd, feedback
import numpy as np
from scipy.integrate import odeint
from time import sleep
import select


x = None
y = None
theta = None
v = 0
phi = 0
l = None
xs = np.array([])
ys = np.array([])
thetas = np.array([])
stop = False

def init_handler(channel, data):
    msg = init_state.decode(data)
    globals()['x'] = msg.xi
    globals()['y'] = msg.yi
    globals()['theta'] = msg.thetai
    globals()['l'] = msg.l
    if msg.planning is False:
        globals()['x'] -= msg.xf
        globals()['y'] -= msg.yf
    print("INIT PLANT")


def control_handler(channel, data):
    msg = control_cmd.decode(data)
    globals()['v'] = msg.v
    globals()['phi'] = msg.phi


def system(state, t, v, phi):
    _, _, theta = state
    dtheta = v/l*np.tan(phi)
    return v*np.cos(theta), v*np.sin(theta), dtheta


def terminate_handler(channel, data):
    globals()['stop'] = True
    print("TERMINATE PLANT")


lc = lcm.LCM()
lc.subscribe("INIT", init_handler)
lc.subscribe("CONTROL", control_handler)
lc.subscribe("TERMINATE", terminate_handler)
lc.handle()
timeout = 0
while not stop:
    rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
    if rfds:
        lc.handle()
    t = np.arange(0, 0.0005, 0.0001)
    sol = odeint(system, [x, y, theta], t, args=(v, phi,))
    x = sol[-1, 0]
    y = sol[-1, 1]
    theta = sol[-1, 2]
    msg = feedback()
    msg.x = x
    msg.y = y
    msg.theta = theta
    lc.publish("FEEDBACK", msg.encode())
    sleep(0.0005)
