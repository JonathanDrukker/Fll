import micropython
from os import system
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button
from pybricks.media.ev3dev import Image
from mytools import thread
from time import sleep


class Handler:
    """
    Handler class.
    """

    pressed = []

    def __init__(self, runner: object):
        self.runner = runner
        self.ev3 = EV3Brick()

    def main(self):
        """
        Main function.
        """
        self.startExit()

        self.ev3.screen.clear()
        self.ev3.screen.load_image(Image('/home/robot/Robot/Media/RunButtons.png'))

        self.runner.drivebase.update()

        print("Choose a path:")

        while True:

            pressed = self.ev3.buttons.pressed()

            if pressed:

                if pressed[0] == Button.LEFT_UP:
                    break

                if pressed[0] == Button.CENTER:
                    log, count = self.runner.path('1', 0.175, 1, True)

                    with open('/home/robot/Logs/runtime1.log', 'w') as f:
                        f.write(str(log))
                    print("Count 1:", count)

                elif pressed[0] == Button.LEFT:
                    log, count = self.runner.path('2', 0.175, 1, True)

                    with open('/home/robot/Logs/runtime2.log', 'w') as f:
                        f.write(str(log))
                    print("Count 2:", count)

                elif pressed[0] == Button.UP:
                    log, count = self.runner.path('3', 0.175, 1, True)

                    with open('/home/robot/Logs/runtime3.log', 'w') as f:
                        f.write(str(log))
                    print("Count 3:", count)

                elif pressed[0] == Button.RIGHT:
                    log, count = self.runner.path('4', 0.175, 1, True)

                    with open('/home/robot/Logs/runtime4.log', 'w') as f:
                        f.write(str(log))
                    print("Count 4:", count)

                elif pressed[0] == Button.DOWN:
                    log, count = self.runner.path('5', 0.175, 1, True)

                    with open('/home/robot/Logs/runtime5.log', 'w') as f:
                        f.write(str(log))
                    print("Count 5:", count)

    @thread
    def startExit(self):
        """
        Start exit thread.
        """

        while True:

            pressed = self.ev3.buttons.pressed()
            if pressed and pressed[0] == Button.LEFT_UP:
                break

            sleep(0.25)

        system('nice -n -20 bash /home/robot/Commands/terminate.sh')
