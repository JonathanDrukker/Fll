from json import load
from math import degrees, sin, cos


# {'x': x, 'y': y, 'theata': theata}

DTA = 6

print("Starting...")
with open('deploy\pathplanner\generatedJSON\Test.wpilib.json', 'r') as f:

    path = load(f)
    waypoints = []

    for dict in path:

        theata = dict['pose']['rotation']['radians']
        x = -DTA*cos(theata) + dict['pose']['translation']['x']
        y = -DTA*sin(theata) + dict['pose']['translation']['y']

        waypoint = {'time': dict['time'],
                    'pose': (x*100, y*100, degrees(theata))}
        waypoints.append(waypoint)

    with open('Robot\Test.waypoints', 'w') as f:
        f.write(str(waypoints))

print("Done!")
