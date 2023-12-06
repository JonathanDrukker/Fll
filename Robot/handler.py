import micropython
from pybricks.hubs import EV3Brick
from mytools import thread
from time import sleep


class Handler:
    def __init__(self, runner: object):
        self.runner = runner
        self.ev3 = EV3Brick()

    @micropython.native
    @thread
    def startExit(self):
        self.run = True
        while self.run and not self.ev3.buttons.pressed():
            sleep(0.25)
        if self.run:
            self.runner.exit()

    @micropython.native
    def stopExit(self):
        self.run = False
