from json import load
from math import sin, cos, pi
from Tools.graph import graph as _graph


l = 2.4 # noqa
DBM = 9.7
halfDBM = DBM/2

graph = True

print("Starting...")
with open('deploy\pathplanner\generatedJSON\Test.wpilib.json', 'r') as f:

    path = load(f)
    waypoints = []

    if graph:
        X, Y = [], []

    for index, dict in enumerate(path):

        theata = dict['pose']['rotation']['radians']
        if theata < 0: theata += 2*pi
        x = dict['pose']['translation']['x']*100 + l*sin(theata)
        y = dict['pose']['translation']['y']*100 + l*cos(theata)

        V = dict['velocity']*100
        omega = dict['angularVelocity']

        if index == len(path)-1: leftAccel, rightAccel = dict['acceleration'], dict['acceleration']
        else:
            Vl, Vr = V - omega*halfDBM, V + omega*halfDBM
            nVl = path[index+1]['velocity']*100 - path[index+1]['angularVelocity']*halfDBM
            nVr = path[index+1]['velocity']*100 + path[index+1]['angularVelocity']*halfDBM
            dt = path[index+1]['time'] - dict['time']
            if dt == 0: pass
            else:
                leftAccel = (nVl - Vl) / dt
                rightAccel = (nVr - Vr) / dt

        waypoint = {'time': dict['time'], 'x': x, 'y': y, 'theata': theata,
                    'V': V, 'omega': omega, 'leftAcc': leftAccel, 'rightAcc': rightAccel}
        waypoints.append(waypoint)

        if graph:
            X.append(x)
            Y.append(y)

    with open('Robot\paths.py', 'w') as f:
        f.write('path = '+str(waypoints))

if graph:
    _graph({'x0': X, 'y0': Y, 'c0': 'hsv', 'title': 'Planned Path', 'lables': ['X', 'Y']})

print("Done!")
