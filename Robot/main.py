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
                      8.16/2, 9.5, 0, 0, 0)


with open('Comms/ipAdd', 'r') as f:
    ip = f.read()

print("Connecting to server... IP: ", ip)
client = Client(ip, 5000)
print("Connected to server! IP: ", ip)

with open('Test.waypoints', 'r') as f:
    data = eval(f.read())
    drivebase.trackPath(data, 0.05, 0.1, True, client)

with drivebase.lock:
    client.send(Message('showGraph', ''))

drivebase.run_tank(0, 0)

sleep(1)
with drivebase.lock:
    client.quit()
print("Done!")
