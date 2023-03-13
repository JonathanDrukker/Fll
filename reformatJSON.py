from json import load
from math import degrees


# {'x': x, 'y': y, 'theata': theata}

print("Starting...")
with open('deploy\pathplanner\generatedJSON\Test.wpilib.json', 'r') as f:
    path = load(f)
    waypoints = []
    for dict in path:
        waypoint = {'time': dict['time'],
                    'pose': (dict['pose']['translation']['x']*100,
                             dict['pose']['translation']['y']*100,
                             degrees(dict['pose']['rotation']['radians']))}
        waypoints.append(waypoint)

    with open('Robot\Test.waypoints', 'w') as f:
        f.write(str(waypoints))

print("Done!")
