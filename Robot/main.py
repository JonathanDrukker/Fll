#!/home/robot/Robot/micropython

import micropython; micropython.opt_level(3)
from runner import Runner
from config import config

print("Starting...")

runner = Runner(config())

runner.drivebase.update()

log, count = runner.path('6', 0.01, 0.4, True)

runner.drivebase.stopUpdate()

print("Count:", count)

with open('runtime.log', 'w') as f:
    f.write(str(log))

print("Done!")
