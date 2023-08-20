#!/home/robot/Robot/micropython

from paths import path
from runner import Runner
from robots import robot

print("Starting...")

runner = Runner(("MD", {}),
                ("MB", {}),
                ("S3", "S4", True, {}),
                robot.wheelRad, robot.DBM)


log, count = runner.path(path, 0.01, 0.5, 0, True)

with open('runtime.log', 'w') as f:
    f.write(str(log))

print("Done!")
