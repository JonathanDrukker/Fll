dict = {}

print("Starting...")
with open('Robot\path.waypoints', 'r') as f:

    path = eval(f.read())
    x, y = [], []

    for pos in path:

        x.append(pos['pose'][0])
        y.append(pos['pose'][1])

    dict['x0'] = x
    dict['y0'] = y

with open('Robot\Test.waypoints', 'r') as f:

    path = eval(f.read())
    x, y = [], []

    for pos in path:

        x.append(pos['pose'][0])
        y.append(pos['pose'][1])

    dict['x1'] = x
    dict['y1'] = y

dict['title'] = "Path"
dict['lables'] = ("X", "Y")

with open('Tools\graph.data', 'w') as f:
    f.write(str(dict))

print("Done!")
