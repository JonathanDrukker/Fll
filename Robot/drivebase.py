import micropython
from _thread import LockType
from math import pi
from ev3devices import Motor, DualGyro
from typing import List, NoReturn


class DriveBase:
    def __init__(self, lMotor: tuple, rMotor: tuple, gyro: tuple,
                 wheelRadius: float, DBM: float, lock: LockType):

        self.lock = lock

        self.lm = Motor(*lMotor)
        self.rm = Motor(*rMotor)
        self.gyro = DualGyro(*gyro)

        self._wheelRad = micropython.const(wheelRadius)
        self._wheelDiameter = micropython.const(wheelRadius*2)
        self._wheelCircumference = micropython.const(pi*self._wheelDiameter)

        self._DBM = micropython.const(DBM)
        self._halfDBM = micropython.const(DBM/2)

    @micropython.native
    def run_tank(self, Vl, Vr) -> NoReturn:
        self.lm.run(Vl)
        self.rm.run(Vr)

    @micropython.native
    def stop(self) -> NoReturn:
        self.lm.stop()
        self.rm.stop()

    @micropython.native
    def motorSpeed(self, V: float) -> float:
        return V/self._wheelCircumference*360

    @micropython.native
    def getAngle(self) -> List[float, float]:
        return self.lm.getAngle(), self.rm.getAngle()

    @micropython.native
    def getRot(self) -> List[float, float]:
        return self.lm.getRot(), self.rm.getRot()

    @micropython.native
    def getSpeed(self) -> List[float, float]:
        return self.lm.getSpeed(), self.rm.getRot()

    @micropython.native
    def reset(self) -> NoReturn:
        self.lm.reset(); self.rm.reset()
