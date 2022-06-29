from sympy import *
import lcm

from car import init_state, planning_cmd, terminate
import numpy as np
from time import sleep, perf_counter
import select

t0 = perf_counter()
lc = lcm.LCM()
x = None
y = None
start = False
stop = False

def trajectory(xi, xf, alpha, beta):
    return lambda s: s**3*xf - (s - 1)**3*xi + alpha*s**2*(s - 1) + beta*s*(s - 1)**2


def init_handler(channel, data):
    msg = init_state.decode(data)
    k = msg.k
    xi = msg.xi
    xf = msg.xf
    yi = msg.yi
    yf = msg.yf
    thetai = msg.thetai
    thetaf = msg.thetaf
    alphax = k*np.cos(thetaf) - 3*xf
    alphay = k*np.sin(thetaf) - 3*yf
    betax = k*np.cos(thetai) + 3*xi
    betay = k*np.sin(thetai) + 3*yi
    globals()['x'] = trajectory(xi, xf, alphax, betax)
    globals()['y'] = trajectory(yi, yf, alphay, betay)
    globals()['start'] = True

def terminate_handler(channel, data):
    globals()['stop'] = True

lc.subscribe("INIT", init_handler)
lc.subscribe("TERMINATE", terminate_handler)
timestamp = 0
t = symbols('t')
lc.handle()
dx = x(t).diff(t)
dy = y(t).diff(t)
ddx = dx.diff(t)
ddy = dy.diff(t)
thetad = atan2(dy, dx)
vd = sqrt(dx**2+dy**2)
print(vd)
dthetad = (ddy*dx-ddx*dy)/(dx**2+dy**2)
timeout = 0
while not stop:
    rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
    if rfds:
        lc.handle()
    msg = planning_cmd()
    msg.timestamp = timestamp
    msg.xd = x(timestamp)
    msg.yd = y(timestamp)
    msg.vd = vd(timestamp)
    msg.thetad = thetad(timestamp)
    msg.dthetad = dthetad(timestamp)
    lc.publish("PLANNING", msg.encode())
    timestamp += 0.02
    sleep(0.02)
