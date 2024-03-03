from os import system
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color
from pybricks.media.ev3dev import Image
from mytools import thread
from time import time, sleep


class Handler:
    """
    Handler class.
    """

    running = False

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

        lA = self.runner.lm.getAngle()
        rA = self.runner.rm.getAngle()
        Kp = 20

        print("Choose a path:")
        self.ev3.light.on(Color.ORANGE)
        sleep(0.5)
        self.ev3.light.on(Color.GREEN)

        while True:

            pressed = self.ev3.buttons.pressed()

            sign = 1 if int(time()*10) % 2 == 0 else -1
            self.runner.lm.runImmediate(60*sign + Kp*(lA - self.runner.lm.getAngle()))
            self.runner.rm.runImmediate(60*sign + Kp*(rA - self.runner.rm.getAngle()))

            if pressed:

                self.running = True

                self.runner.lm.dutyCycle(0)
                self.runner.rm.dutyCycle(0)

                if pressed[0] == Button.CENTER:
                    log, count = self.runner.path('1', 0.025, 0.5, True)

                    with open('/home/robot/Logs/runtime1.log', 'w') as f:
                        f.write(str(log))
                    print("Count 1:", count)

                elif pressed[0] == Button.UP:
                    log, count = self.runner.path('2', 0.025, 0.7, True)

                    with open('/home/robot/Logs/runtime2.log', 'w') as f:
                        f.write(str(log))
                    print("Count 2:", count)

                elif pressed[0] == Button.RIGHT:
                    log, count = self.runner.path('3', 0.05, 0.6, True)

                    with open('/home/robot/Logs/runtime3.log', 'w') as f:
                        f.write(str(log))
                    print("Count 3:", count)

                    self.running = False

                    while not self.ev3.buttons.pressed():
                        sleep(0.1)

                    self.runner.drivebase.gyro.reset()
                    self.runner.sensorbase.Drive(400, 30, 0, 10, 0.05, 2, 3)
                    self.runner.drivebase.run_tank(-1000, -700)
                    sleep(0.5)
                    self.runner.drivebase.stop()

                elif pressed[0] == Button.DOWN:
                    log, count = self.runner.path('4', 0.025, 0.7, True)

                    with open('/home/robot/Logs/runtime4.log', 'w') as f:
                        f.write(str(log))
                    print("Count 4:", count)

                elif pressed[0] == Button.LEFT:
                    log, count = self.runner.path('5', 0.045, 0.7, True)

                    with open('/home/robot/Logs/runtime5.log', 'w') as f:
                        f.write(str(log))
                    print("Count 5:", count)

                self.running = False

                rA = self.runner.rm.getAngle()
                lA = self.runner.lm.getAngle()

                sleep(1)

    @thread
    def startExit(self):
        """
        Start exit thread.
        """

        while True:

            pressed = self.ev3.buttons.pressed()
            if pressed:
                if pressed[0] == Button.LEFT_UP:
                    break
                elif self.running:
                    self.runner.exit()
                    sleep(0.5)
                    self.runner.drivebase.update()

            sleep(0.2)

        system('nice -n -20 bash /home/robot/Commands/terminate.sh')
