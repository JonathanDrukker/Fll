from json import load
from math import sin, cos, pi


DTA = -2
wheelRadius = 8.16/2
wheelCircumference = 2*pi*wheelRadius

print("Starting...")
with open('deploy\pathplanner\generatedJSON\Test.wpilib.json', 'r') as f:

    path = load(f)
    waypoints = []

    for dict in path:

        theata = dict['pose']['rotation']['radians']
        x = dict['pose']['translation']['x']*100 - DTA*sin(theata)
        y = dict['pose']['translation']['y']*100 - DTA*cos(theata)

        V = dict['velocity']*100
        omega = dict['angularVelocity']

        Vl = V/wheelCircumference*360 - omega
        Vr = V/wheelCircumference*360 + omega

        waypoint = {'time': dict['time'],
                    'x': x, 'y': y, 'theata': theata, 'Vl': Vl, 'Vr': Vr}
        waypoints.append(waypoint)

    with open('Robot\Test.waypoints', 'w') as f:
        f.write(str(waypoints))

print("Done!")
