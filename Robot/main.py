#!/home/robot/Robot/micropython

from paths import path_1
from runner import Runner
from config import config

print("Starting...")

runner = Runner(config())
runner.odometry.start()

log, count = runner.path(path_1, 0.01, 0.5, 0, True)

with open('runtime.log', 'w') as f:
    f.write(str(log))

print("Done!")
