from json import load
from math import degrees, sin, cos


DTA = 6

print("Starting...")
with open('deploy\pathplanner\generatedJSON\Test.wpilib.json', 'r') as f:

    path = load(f)
    waypoints = []

    for dict in path:

        theata = dict['pose']['rotation']['radians']
        x = dict['pose']['translation']['x']*100 - DTA*sin(theata)
        y = dict['pose']['translation']['y']*100 - DTA*cos(theata)

        waypoint = {'time': dict['time'],
                    'pose': (x, y, degrees(theata))}
        waypoints.append(waypoint)

    with open('Robot\Test.waypoints', 'w') as f:
        f.write(str(waypoints))

print("Done!")
