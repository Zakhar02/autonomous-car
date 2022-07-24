import lcm
from car import feedback, init_state, control_cmd, terminate, planning_cmd
import numpy as np
from time import sleep
import select


x = None
y = None
theta = None
l = None
stop = False
start = False
planning = None
xd = None
yd = None
vd = None
thetad= None
dthetad = None
k1 = 1
k2 = 5
zeta = 1
a = 10

def init_handler(channel, data):
    msg = init_state.decode(data)
    globals()['l'] = msg.l
    globals()['x'] = msg.xi
    globals()['y'] = msg.yi
    globals()['theta'] = msg.thetai
    globals()['start'] = True
    globals()['planning'] = msg.planning
    if msg.planning is False:
        globals()['x'] -= msg.xf
        globals()['y'] -= msg.yf

def control_handler(channel, data):
    msg = feedback.decode(data)
    globals()['x'] = msg.x
    globals()['y'] = msg.y
    globals()['theta'] = msg.theta

def terminate_handler(channel, data):
    globals()['stop'] = True

def planning_handler(channel, data):
    msg = planning_cmd.decode(data)
    globals()['xd'] = msg.xd
    globals()['yd'] = msg.yd
    globals()['vd'] = msg.vd
    globals()['thetad'] = msg.thetad
    globals()['dthetad'] = msg.dthetad


lc = lcm.LCM()
lc.subscribe("INIT", init_handler)
lc.subscribe("FEEDBACK", control_handler)
lc.subscribe("TERMINATE", terminate_handler)
lc.subscribe("PLANNING", planning_handler)
lc.handle()
timeout = 0
while not stop:
    rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
    if rfds:
        lc.handle()
    msg = control_cmd()
    if not planning:
        if abs(x) < 0.1 and abs(y) < 0.1:
            lc.publish("TERMINATE", terminate().encode())
            break
        msg.v = -k1*(x*np.cos(theta) + y*np.sin(theta))
        msg.phi = np.arctan2(k2*l*(np.arctan2(y, x) - theta + np.pi), msg.v)
    else:
        E = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
        qd = np.array([[xd - x, yd - y, thetad - theta]]).T
        e = (E@qd).T[0]
        k1 = 2*zeta*a
        u1 = -k1*e[0]
        msg.v = vd*np.cos(e[2]) - u1
        k2 = (a**2 - dthetad**2)/vd if vd !=0 else a**2
        u2 = -k2*e[1] - k1*e[2]
        msg.phi = np.arctan2(l*(dthetad - u2), msg.v)
    lc.publish("CONTROL", msg.encode())
    sleep(0.005)
