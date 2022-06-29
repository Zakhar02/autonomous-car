import configparser
from car import init_state
import lcm


config = configparser.ConfigParser()
config.read('start.cfg')
msg = init_state()
config = config['DATA']
msg.xi = float(config['xi'])
msg.xf = float(config['xf'])
msg.yf = float(config['yf'])
msg.yi = float(config['yi'])
msg.l = float(config['l'])
msg.thetai = float(config['thetai'])
msg.thetaf = float(config['thetaf'])
msg.k = float(config['k'])
msg.planning = bool(int(config['planning']))
lc = lcm.LCM()
lc.publish("INIT", msg.encode())
print("SENT")
