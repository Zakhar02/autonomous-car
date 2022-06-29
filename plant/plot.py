import lcm
from car import feedback
import numpy as np
import select
import matplotlib.pyplot as plt


x = np.array([])
y = np.array([])
theta = np.array([])
stop = False

def my_handler(channel, data):
    msg = feedback.decode(data)
    globals()['x'] = np.append(globals()['x'], msg.x)
    globals()['y'] = np.append(globals()['y'], msg.y)
    globals()['theta'] = np.append(globals()['theta'], msg.theta)

def terminate_handler(channel, data):
    globals()['stop'] = True


lc = lcm.LCM()
lc.subscribe("FEEDBACK", my_handler)
lc.subscribe("TERMINATE", terminate_handler)
lc.handle()
timeout = 0
while not stop:
    rfds, _, _ = select.select([lc.fileno()], [], [], timeout)
    if rfds:
        lc.handle()
plt.plot(x, y)
plt.show()
