#!/home/robot/Robot/micropython.sh

import micropython; micropython.opt_level(3)
from runner import Runner
from config import config
from handler import Handler
from time import sleep

print("Starting...")

runner = Runner(config())
handler = Handler(runner)

# handler.main()

handler.startExit()
runner.drivebase.update()

runner.path('6', 0.02, 0.7)

sleep(8)

runner.path('7', 0.02, 0.7)

print("Done!")
