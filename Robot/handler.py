import micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button
from mytools import thread
from time import sleep


class Handler:
    def __init__(self, runner: object):
        self.runner = runner
        self.ev3 = EV3Brick()

    @micropython.native
    def main(self):
        """
        Main function.
        """

        count = log = [[] for _ in range(5)]

        self.runner.drivebase.update()

        while True:
            pressed = self.ev3.buttons.pressed()[0] or [None]

            if pressed[0] is not None:
                self.startExit()

                if len(pressed) > 1:
                    break

                elif pressed[0] == Button.CENTER:
                    log[0], count[0] = self.runner.path('1', 0.175, 1, True)

                elif pressed[0] == Button.LEFT:
                    log[1], count[1] = self.runner.path('2', 0.175, 1, True)

                elif pressed[0] == Button.UP:
                    log[2], count[2] = self.runner.path('3', 0.175, 1, True)

                elif pressed[0] == Button.RIGHT:
                    log[3], count[3] = self.runner.path('4', 0.175, 1, True)

                elif pressed[0] == Button.DOWN:
                    log[4], count[4] = self.runner.path('5', 0.175, 1, True)

            else:
                self.stopExit()

        self.runner.exit()

        for i in range(5):
            with open('runtime'+str(i)+'.log', 'w') as f:
                f.write(str(log[i]))
                print("Count", str(i)+":", count[i])

    @micropython.native
    @thread
    def startExit(self):
        """
        Start exit thread.
        """

        self.runExit = True
        while self.runExit and not self.ev3.buttons.pressed():
            sleep(0.25)

        if self.run:
            self.runner.exit()

    @micropython.native
    def stopExit(self):
        """
        Stop exit thread.
        """
        self.runExit = False
