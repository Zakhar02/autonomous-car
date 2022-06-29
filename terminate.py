from car import terminate
import lcm

lc = lcm.LCM()
lc.publish("TERMINATE", terminate().encode())
