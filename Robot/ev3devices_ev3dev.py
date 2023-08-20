from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, Motor as _Motor  # noqa
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4  # noqa
from ev3dev2.sensor.lego import GyroSensor as _GyroSensor, ColorSensor as _ColorSensor
from math import radians
import micropython


@micropython.native
def Motor(port: str, kwargs: dict):
    temp = _Motor(__eval_port(port))
    for key, value in kwargs.items():
        setattr(temp, key, value)
    return temp


@micropython.native
def Gyro(port: str, kwargs: dict):
    temp = _GyroSensor(__eval_port(port))
    for key, value in kwargs.items():
        temp.__setattr__(key, value)
    return temp


class DualGyro:
    def __init__(self, port1, port2, inverse=1, kwargs={}, offset=0):
        self.gyro1 = Gyro(port1, kwargs)
        self.gyro2 = Gyro(port2, kwargs)
        self.offset = offset
        self._k = micropython.const(2*inverse)

    @micropython.native
    def getRawAngle(self):
        return (self.gyro1.angle + self.gyro2.angle)/self._k + self.offset

    @micropython.native
    def getAngle(self):
        angle = self.getRawAngle() % 360
        return angle if angle > 0 else angle + 360

    @micropython.native
    def getRadians(self):
        return radians(self.getAngle())

    @micropython.native
    def getRawRadians(self):
        return radians(self.getRawAngle())

    @micropython.native
    def getSpeed(self):
        return (self.gyro1.rate + self.gyro2.rate)/self._avg

    @micropython.native
    def getSpeedRadians(self):
        return radians(self.getSpeed())

    @micropython.native
    def reset(self, angle: float):
        self.gyro1.reset()
        self.gyro2.reset()
        self.offset = angle

    @micropython.native
    def calibrate(self):
        self.gyro1.calibrate()
        self.gyro2.calibrate()


@micropython.native
def ColorSensor(port: str, kwargs: dict):
    temp = _ColorSensor(__eval_port(port))
    for key, value in kwargs.items():
        temp.__setattr__(key, value)
    return temp


@micropython.native
def __eval_port(port: str):
    if port[0] == 'M':
        return eval('OUTPUT_'+port[1])
    else:
        return eval('INPUT_'+port[1])


@micropython.native
def motor_motion_profile(motor: _Motor):
    while True:
        motor.duty_cycle_sp = 100
        motor.command = 'run-direct'
