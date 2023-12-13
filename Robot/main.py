#!/home/robot/Robot/micropython

import micropython; micropython.opt_level(3)
from runner import Runner
from config import config
from handler import Handler

print("Starting...")

runner = Runner(config())
handler = Handler(runner)

handler.main()

print("Done!")
