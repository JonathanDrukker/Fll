import statsmodels.api as sm
import matplotlib.pyplot as plt
import numpy as np
from math import copysign


print("Starting...")

graph = True
path = "Data/Analysis/analysis.log"

with open(path, 'r') as f:

    data_time = {"time": [], "duty": [], "speed": [], "acceleration": []}

    for lst in eval(f.read()):
        data_time["time"].append(lst[0])
        data_time["duty"].append(lst[1])
        data_time["speed"].append(lst[2])

lastTime, lastVel = data_time["time"][0], data_time["speed"][0]

for time, vel in zip(data_time["time"][1:], data_time["speed"][1:]):

    dt = time - lastTime
    if dt != 0:
        data_time["acceleration"].append((vel - lastVel) / (time - lastTime))
    else:
        data_time["acceleration"].append(data_time["acceleration"][-1])

    lastTime, lastVel = time, vel

data_time["acceleration"].insert(0, data_time["acceleration"][0])

data_duty = {"time": [], "duty": [], "speed": [], "acceleration": []}
for duty in range(0, 101):
    startIndex = data_time["duty"].index(duty)
    if duty != 100:
        endIndex = data_time["duty"].index(duty+1)-1
    else:
        endIndex = len(data_time["duty"])-1

    vels = data_time["speed"][startIndex:endIndex]
    for i in range(len(vels)-3):
        vels[i] = sum(vels[i:i+3]) / 3

    vel = max(vels)
    index = startIndex+vels.index(vel)
    data_duty["time"].append(data_time["time"][index])
    data_duty["duty"].append(duty)
    data_duty["speed"].append(vel)
    data_duty["acceleration"].append(
        copysign(data_time["acceleration"][index], duty))

Ks = max([i for i, x in enumerate(data_duty["speed"]) if x == 0])

x1 = np.array(data_duty["speed"][Ks:])
x2 = np.array(data_duty["acceleration"][Ks:])
y = np.array(data_duty["duty"][Ks:])

x = np.array((np.sign(x1), x1, x2)).T
fit = sm.OLS(y, x).fit()
_Ks, Kv, _Ka = fit.params

Ka = 1/((data_duty["acceleration"][-1] - data_duty["acceleration"][Ks]) /
        (data_duty["time"][-1]-data_duty["time"][Ks]))

print(Ks, Kv, Ka)

if graph:

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 12))

    ax1.plot(data_duty["duty"], data_duty["speed"])
    ax1.plot([Kv * x for x in data_duty["speed"]], data_duty["speed"])
    ax1.set_title('Duty-Velocity')

    ax2.plot(data_duty["duty"], data_duty["acceleration"])
    ax2.plot([Ka * x for x in data_duty["acceleration"]],
             data_duty["acceleration"])
    ax2.set_title('Duty-Acceleration')

    ax3.plot(data_time["time"], data_time["speed"])
    ax3.plot(data_duty["time"], data_duty["acceleration"])
    ax3.set_title('Time-Velocity')

    plt.tight_layout()
    plt.show()

print("Done!")
