from math import sin, cos, pi
from json import loads
import csv
import os
mainpath = os.path.dirname(os.getcwd())


filename = input("Enter file name without the file extension: ")

path_to_JSON = mainpath + f'\FLL\deploy\pathplanner\generatedJSON\{filename}.wpilib.json'
path_to_Rfile = mainpath + f'\FLL\deploy\pathplanner\{filename}.path'
path_to_Wfile = mainpath + f'\FLL\Robot\Paths\{filename}'

resolution = 0.001
unitsScale = 100

with open("Robot/config.json", "r") as f:
    config = loads(f.read())

with open(path_to_Rfile, 'r') as f:
    points = loads(f.read())
with open(path_to_JSON, 'r') as f:
    waypoints = loads(f.read())

# {"Time": [], 'x': [], 'y': [], "theata": [], "V": [], "omega": [], "accL": [], "accR": []}
path = []
segment = []
new_waypoint = {}

checkStopPoint = False

Vl, Vr = 0, 0

# Waypoints

for index, waypoint in enumerate(waypoints):

    if checkStopPoint and waypoint['velocity'] == 0 and len(segment) > 0:
        path.append(segment)
        segment = []
        checkStopPoint = False
    else:
        checkStopPoint = True

    # Time
    new_waypoint["time"] = waypoint["time"]

    # Coords
    x, y = waypoint['pose']['translation']['x']*unitsScale, waypoint['pose']['translation']['y']*unitsScale
    l = config["robot"]["length"]/2 - config["robot"]["wheelAxis"]  # noqa
    newX, newY = x + l*cos(waypoint['pose']['rotation']['radians']), y + l*sin(waypoint['pose']['rotation']['radians'])

    new_waypoint["x"] = newX
    new_waypoint["y"] = newY

    # Theata
    theata = waypoint['pose']['rotation']['radians']
    if (theata < 0):
        theata += 2*pi
    new_waypoint["theata"] = theata

    # Velocity
    V = waypoint['velocity']*unitsScale
    new_waypoint["V"] = V

    # Omega
    omega = waypoint['angularVelocity']
    new_waypoint["omega"] = omega

    # Acc
    if index == len(waypoints)-1:
        new_waypoint["accL"] = 0
        new_waypoint["accR"] = 0
    else:
        nextWaypoint = waypoints[index+1]
        dt = nextWaypoint['time'] - waypoint['time']

        if dt == 0:
            if len(segment) > 0:
                new_waypoint["accL"] = segment['accL'][-1]
                new_waypoint["accR"] = segment['accR'][-1]
            else:
                new_waypoint["accL"] = path[-1][-1]['accL']
                new_waypoint["accR"] = path[-1][-1]['accR']

        else:
            nextV = nextWaypoint['velocity']*unitsScale
            nextOmega = nextWaypoint['angularVelocity']
            nextVl = nextV - nextOmega*config["drivebase"]["halfDBM"]
            nextVr = nextV + nextOmega*config["drivebase"]["halfDBM"]

            accL = (nextVl - Vl) / dt
            accR = (nextVr - Vr) / dt

            new_waypoint["accL"] = accL
            new_waypoint["accR"] = accR

            Vl, Vr = nextVl, nextVr

    segment.append(new_waypoint)
    new_waypoint = {}

# Stop Events

stopEvents = []
for point in points["waypoints"]:
    if point["isStopPoint"]:

        commands = []
        for index, i in enumerate(point["stopEvent"]["names"][::2]):
            commands.append(f"{i}{point['stopEvent']['names'][index*2+1]}")
        executionBehavior = point["stopEvent"]["executionBehavior"]

        waitTime = point["stopEvent"]["waitTime"]
        waitBehavior = point["stopEvent"]["waitBehavior"]

        stopEvents.append({"commands": commands, "executionBehavior": executionBehavior,
                           "waitTime": waitTime, "waitBehavior": waitBehavior})

# Markers

markers = {}
for marker in points["markers"]:

    commands = []
    for index, i in enumerate(marker["names"][::2]):
        commands.append(f"{i}{marker['names'][index*2+1]}")

    spline_index = int(marker['position'])
    time = path[spline_index][int(round(marker['position'], 3) * (1/resolution))]

    markers[time] = commands

# Reversed Path
isReversed = True if points['isReversed'] else False

with open(path_to_Wfile+".txt", 'w') as f:
    f.write(f"""{dict(stopEvents=stopEvents, markers=markers, reversed=isReversed)}""")

with open(path_to_Wfile+".csv", 'w', newline='') as f:
    w = csv.DictWriter(f, path[0][0].keys())
    w.writeheader()
    for segment in path:
        for waypoint in segment:
            w.writerow(waypoint)
        w.writerow({"time": "None", 'x': "None", 'y': "None", "theata": "None",
                   "V": "None", "omega": "None", "accL": "None", "accR": "None"})
