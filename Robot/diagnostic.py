#!/home/robot/Robot/micropython

from runner import Runner
from config import config
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color
from time import time, sleep


class GyroOscillation(Exception):
    pass


class BatteryVoltageLow(Exception):
    pass


class PortError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


def main():

    EV3 = EV3Brick()
    EV3.light.on(Color.ORANGE)

    runner = Runner(config())

    try:
        if EV3.battery.voltage() < 7.9:
            raise BatteryVoltageLow

        st = time()
        sValue = runner.drivebase.gyro.getAngle()
        while time()-st < 10:
            if runner.drivebase.gyro.getAngle() != sValue:
                raise GyroOscillation

        sValue = runner.drivebase.getAngle()
        runner.drivebase.run_tank(100, 100)
        sleep(2)
        runner.drivebase.stop()
        if runner.drivebase.getAngle()[0] == sValue[0]:
            raise PortError("LargeLeft")
        elif runner.drivebase.getAngle()[1] == sValue[1]:
            raise PortError("LargeRight")

        sValue = runner.lm.getAngle(), runner.rm.getAngle()
        runner.lm.run(100); runner.rm.run(100)
        sleep(2)
        runner.lm.stop(); runner.rm.stop()
        if runner.lm.getAngle() == sValue[0]:
            raise PortError("SmallLeft")
        elif runner.rm.getAngle() == sValue[1]:
            raise PortError("SmallRight")

    except GyroOscillation:
        EV3.light.on(Color.RED)
        EV3.screen.print("Gyro Oscillation")
        EV3.screen.print("Calibrate the gyro")
        runner.drivebase.gyro.calibrate()
        EV3.screen.print("Finished!")
        EV3.speaker.beep()

    except BatteryVoltageLow:
        EV3.light.on(Color.RED)
        EV3.screen.print("Battery Voltage Low")
        EV3.screen.print("Replace the batteries")
        EV3.speaker.beep()

    except PortError as e:
        EV3.light.on(Color.RED)
        EV3.screen.print("Port Error on: ")
        EV3.screen.print(e.value)
        EV3.speaker.beep()

    else:
        EV3.light.on(Color.GREEN)
        EV3.screen.print("No Errors")
        EV3.speaker.beep()


if __name__ == '__main__':
    main()
