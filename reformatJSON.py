from json import load
from math import sin, cos, pi
from Tools.graph import graph as _graph


DTA = 2
wheelRadius = 8.16/2
wheelCircumference = 2*pi*wheelRadius
DBM = 9.5
halfDBM = DBM/2

graph = True

print("Starting...")
with open('deploy\pathplanner\generatedJSON\Test.wpilib.json', 'r') as f:

    path = load(f)
    waypoints = []

    if graph:
        X, Y = [], []

    for dict in path:

        theata = dict['pose']['rotation']['radians']
        x = dict['pose']['translation']['x']*100 + DTA*sin(theata)
        y = dict['pose']['translation']['y']*100 + DTA*cos(theata)

        V = dict['velocity']*100
        omega = dict['angularVelocity']

        Vl = V/wheelCircumference*360 - omega*halfDBM/wheelCircumference*360
        Vr = V/wheelCircumference*360 + omega*halfDBM/wheelCircumference*360

        waypoint = {'time': dict['time'],
                    'x': x, 'y': y, 'theata': theata, 'Vl': Vl, 'Vr': Vr}
        waypoints.append(waypoint)

        if graph:
            X.append(x)
            Y.append(y)

    with open('Robot\Test.waypoints', 'w') as f:
        f.write(str(waypoints))

if graph:
    _graph({'x0': X, 'y0': Y, 'title': 'Planned Path', 'lables': ['X', 'Y']})

print("Done!")
