from json import loads
from Tools.graph import log_graph
from Robot.robots import robot
from math import pi
import os

path = os.path.dirname(os.path.abspath(__file__))

path_name = input("Enter file name without the file extension: ")

path_to_JSON = path + f'\deploy\pathplanner\generatedJSON\{path_name}.wpilib.json'
path_to_Rfile = path + f'\deploy\pathplanner\{path_name}.path'
path_to_Wfile = path + '\Robot\paths.py'

scale = 100
resolution = 0.001

l = robot.WA_to_middle  # noqa
halfDBM = robot.halfDBM

graph = True
wheelRad = robot.wheelRad

with open(path_to_JSON, 'r') as f:
    path = loads(f.read())

waypoints = []

for dict in path:

    x, y = dict['pose']['translation']['x']*scale, dict['pose']['translation']['y']*scale
    theata = dict['pose']['rotation']['radians']
    if (theata < 0):
        theata += 2*pi

    V = dict['velocity']*scale
    omega = dict['angularVelocity']

    waypoint = {'time': dict['time'], 'x': x, 'y': y, 'theata': theata,
                'V': V, 'omega': omega}
    waypoints.append(waypoint)

accL, accR = path[0]['acceleration']*scale, path[0]['acceleration']*scale
waypoints[0]['accL'], waypoints[0]['accR'] = accL, accR

for index, dict in enumerate(waypoints[1:]):

    V = dict['V']
    omega = dict['omega']
    Vl, Vr = V - omega*halfDBM, V + omega*halfDBM

    lastV = waypoints[index]['V']
    lastOmega = waypoints[index]['omega']
    lastVl, lastVr = lastV - lastOmega*halfDBM, lastV + lastOmega*halfDBM

    dt = dict['time'] - waypoints[index]['time']
    if (dt != 0):
        accL = (Vl - lastVl) / dt
        accR = (Vr - lastVr) / dt

    dict['accL'], dict['accR'] = accL, accR

with open(path_to_Rfile, 'r') as f:
    path_bezier = loads(f.read())

markers = {}
for marker in path_bezier['markers']:
    index = int(marker['position'] / resolution)
    if (index > len(waypoints)):
        index = len(waypoints) - 1
    events = marker['names']
    markers[waypoints[index]['time']] = {'command': events[::2], 'parameters': events[1::2]}

for t, point in enumerate(path_bezier['waypoints']):
    if point['isStopPoint']:
        if point['stopEvent']['names']:
            index = int(t / resolution)
            events = point['stopEvent']['names']
            markers[waypoints[index]['time']] = {'command': events[::2],
                                                 'parameters': [eval(i) for i in events[1::2]]}

split_waypoints = []
waypoint_group = []
index = 0
while (index < len(waypoints)):
    dict = waypoints[index]
    waypoint_group.append(dict)
    if (dict['V'] == 0 and len(waypoint_group) > 1):
        split_waypoints.append(waypoint_group)
        waypoint_group = []
        index += 1
    elif (index == len(waypoints) - 1):
        split_waypoints.append(waypoint_group)
        break
    index += 1

with open(path_to_Wfile, 'w') as f:
    f.write(f"path_{path_name} = {str(split_waypoints)}\nmarkers_{path_name} = {str(markers)}")

if graph:
    graph = []
    for dict in waypoints:
        graph.append({'time': dict['time'], 'waypoint': dict})
    log_graph(graph, robot.wheelRad, robot.DBM)
