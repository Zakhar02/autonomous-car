import lcm
from car import feedback, init_state, planning_cmd
import numpy as np
import select
import matplotlib.pyplot as plt


x = np.array([])
y = np.array([])
xd = np.array([])
yd = np.array([])
theta = np.array([])
stop = False
xi = None
xf = None
yi = None
yf = None
planning = None

def init_handler(channel, data):
    msg = init_state.decode(data)
    globals()['xf'] = msg.xf
    globals()['yf'] = msg.yf
    globals()['planning'] = msg.planning
    print("INIT PLOT")

def my_handler(channel, data):
    msg = feedback.decode(data)
    globals()['x'] = np.append(globals()['x'], msg.x)
    globals()['y'] = np.append(globals()['y'], msg.y)
    globals()['theta'] = np.append(globals()['theta'], msg.theta)

def planning_handler(channel, data):
    msg = planning_cmd.decode(data)
    globals()['xd'] = np.append(globals()['xd'], msg.xd)
    globals()['yd'] = np.append(globals()['yd'], msg.yd)

def terminate_handler(channel, data):
    globals()['stop'] = True
    print("TERMINATE PLOT")


lc = lcm.LCM()
lc.subscribe("INIT", init_handler)
lc.subscribe("FEEDBACK", my_handler)
lc.handle()
if planning is True:
    lc.subscribe("PLANNING", planning_handler)
lc.subscribe("TERMINATE", terminate_handler)
timeout = 0
while not stop:
    rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
    if rfds:
        lc.handle()
if planning is False:
    plt.plot(x + xf, y + yf)
else:
    plt.plot(x, y)
    plt.plot(xd, yd)
plt.show()
