import micropython
from _thread import LockType
from math import sin, cos, radians
from mytools import thread
from typing import List, NoReturn


class DiffrentialDriveOdometry:
    def __init__(self, drivebase: object, lock: LockType, x: int = 0, y: int = 0, theata: int = 0):

        self.lock = lock

        self.drivebase = drivebase

        self.x, self.y = x, y
        self.theata = theata

        self.run = False

    @micropython.native
    @thread
    def start(self) -> NoReturn:
        self.run = True

        past_lm_pos, past_rm_pos = self.drivebase.getRot()

        while self.run:

            lm_pos, rm_pos = self.drivebase.getRot()

            Vl = lm_pos - past_lm_pos
            Vr = rm_pos - past_rm_pos

            V = self.drivebase._wheelCircumference/2*(Vr + Vl)
            theata = radians(self.drivebase.gyro.getAngle())

            Vx = V*cos(theata)
            Vy = V*sin(theata)

            with self.lock:
                self.x += Vx
                self.y += Vy
                self.theata = theata

            past_lm_pos, past_rm_pos = lm_pos, rm_pos

    @micropython.native
    def stop(self) -> NoReturn:
        self.run = False

    @micropython.native
    def getPos2d(self) -> List[float, float, float]:
        with self.drivebase.lock:
            return self.x, self.y, self.theata

    @micropython.native
    def resetPos(self, x: float = 0, y: float = 0, theata: float = 0) -> NoReturn:
        with self.drivebase.lock:
            self.x, self.y, self.theata = x, y, theata
        self.drivebase.gyro.resetAngle(theata)
