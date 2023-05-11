#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Direction
from drivebase import DriveBase
from Comms.client import Client
from Comms.message import Message
from time import sleep


print("Starting...")

ev3 = EV3Brick()
drivebase = DriveBase((Port.D, Direction.CLOCKWISE),
                      (Port.B, Direction.CLOCKWISE),
                      (Port.S3, Direction.COUNTERCLOCKWISE,
                      Port.S4, Direction.COUNTERCLOCKWISE),
                      8.16, 9.5, 0, 0, 0)

with open('Comms/ipAdd', 'r') as f:
    ip = f.read()

print("Connecting to server... IP: ", ip)
client = Client(ip, 5000)
print("Connected to server! IP: ", ip)

with open('Test.waypoints', 'r') as f:
    waypoints = eval(f.read())

with open('runtime.log', 'w'): pass
with open('runtime.log', 'a') as logfile:
    logfile.write('(')
    drivebase.trackPath(waypoints, 0.06, 0.1, False, logfile, client)
    logfile.write(')')

with drivebase.lock:
    client.send(Message('show', str((drivebase.wheelDiameter, drivebase.DBM))))

sleep(1)
with drivebase.lock:
    client.quit()
print("Done!")
