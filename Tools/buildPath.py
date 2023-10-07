from math import sin, cos, pi
from json import loads
import os
mainpath = os.path.dirname(os.getcwd())


filename = input("Enter file name without the file extension: ")

path_to_JSON = mainpath + f'\FLL\deploy\pathplanner\generatedJSON\{filename}.wpilib.json'
path_to_Rfile = mainpath + f'\FLL\deploy\pathplanner\{filename}.path'
path_to_Wfile = mainpath + '\FLL\Robot\paths.py'

resolution = 0.001
unitsScale = 100

with open("Robot/config.json", "r") as f:
    config = loads(f.read())

with open(path_to_Rfile, 'r') as f:
    points = loads(f.read())
with open(path_to_JSON, 'r') as f:
    waypoints = loads(f.read())

path = []
segment = {"Time": [], 'x': [], 'y': [], "theata": [], "V": [], "omega": [], "accL": [], "accR": []}

checkStopPoint = False

Vl, Vr = 0, 0

# Waypoints

for index, waypoint in enumerate(waypoints):

    if checkStopPoint and waypoint['velocity'] == 0 and len(segment['Time']) > 0:
        path.append(segment)
        segment = {"Time": [], 'x': [], 'y': [], "theata": [], "V": [], "omega": [], "accL": [], "accR": []}
        checkStopPoint = False
    else:
        checkStopPoint = True

    # Time
    segment["Time"].append(waypoint["time"])

    # Coords
    x, y = waypoint['pose']['translation']['x']*unitsScale, waypoint['pose']['translation']['y']*unitsScale
    l = config["robot"]["length"]/2 - config["robot"]["wheelAxis"]  # noqa
    newX, newY = x + l*cos(waypoint['pose']['rotation']['radians']), y + l*sin(waypoint['pose']['rotation']['radians'])

    segment['x'].append(newX)
    segment['y'].append(newY)

    # Theata
    theata = waypoint['pose']['rotation']['radians']
    if (theata < 0):
        theata += 2*pi
    segment['theata'].append(theata)

    # Velocity
    V = waypoint['velocity']*unitsScale
    segment['V'].append(V)

    # Omega
    omega = waypoint['angularVelocity']
    segment['omega'].append(omega)

    # Acc
    if index == len(waypoints)-1:
        segment['accL'].append(0)
        segment['accR'].append(0)
    else:
        nextWaypoint = waypoints[index+1]
        dt = nextWaypoint['time'] - waypoint['time']

        if dt == 0:
            if len(segment['accL']) > 0:
                segment['accL'].append(segment['accL'][-1])
                segment['accR'].append(segment['accR'][-1])
            else:
                segment['accL'].append(path[-1]['accL'][-1])
                segment['accR'].append(path[-1]['accR'][-1])

        else:
            nextV = nextWaypoint['velocity']*unitsScale
            nextOmega = nextWaypoint['angularVelocity']
            nextVl = nextV - nextOmega*config["drivebase"]["halfDBM"]
            nextVr = nextV + nextOmega*config["drivebase"]["halfDBM"]

            accL = (nextVl - Vl) / dt
            accR = (nextVr - Vr) / dt

            segment['accL'].append(accL)
            segment['accR'].append(accR)

            Vl, Vr = nextVl, nextVr

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
    time = path[spline_index]["Time"][int(round(marker['position'], 3) * (1/resolution))]

    markers[time] = commands

# Reversed Path
isReversed = True if points['isReversed'] else False

with open(path_to_Wfile, 'w') as f:
    f.write(f"reversed_{filename} = {isReversed}\n" +
            f"path_{filename} = {path}\n" +
            f"stopEvents_{filename} = {stopEvents}\n" +
            f"markers_{filename} = {markers}")
