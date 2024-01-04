#!/home/robot/Robot/micropython.sh

import micropython; micropython.opt_level(3)
from runner import Runner
from config import config
from handler import Handler

print("Starting...")

runner = Runner(config())
handler = Handler(runner)

handler.main()

print("Done!")
