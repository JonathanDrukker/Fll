import micropython
from _thread import LockType
from math import sin, cos, radians, pi
from mytools import thread


class DiffrentialDriveOdometry:
    def __init__(self, drivebase: object, lock: LockType, x: int = 0, y: int = 0, theata: int = 0):

        self.lock = lock

        self.drivebase = drivebase

        self.x, self.y = x, y
        self.theata = theata

        self.run = False

        self.lm_phi, self.rm_phi = drivebase.getRot()

    # @micropython.native
    @thread
    def start(self) -> None:
        self.run = True

        past_lm_pos, past_rm_pos = self.drivebase.getRot()
        theata = self.theata

        while self.run:

            self.lm_phi, self.rm_phi = self.drivebase.getRot()

            Vl = self.lm_phi - past_lm_pos
            Vr = self.rm_phi - past_rm_pos

            V = self.drivebase._wheelCircumference*(Vr + Vl)/2
            # theata = radians(self.drivebase.gyro.getAngle())
            theata += (Vr-Vl)*self.drivebase._wheelRad/self.drivebase._DBM
            theata %= 2*pi

            Vx = V*cos(theata)
            Vy = V*sin(theata)

            with self.lock:
                self.x += Vx
                self.y += Vy
                self.theata = theata

            past_lm_pos, past_rm_pos = self.lm_phi, self.rm_phi

    # @micropython.native
    def stop(self) -> None:
        self.run = False

    # @micropython.native
    def getPos2d(self) -> [float, float, float]:
        with self.drivebase.lock:
            return self.x, self.y, self.theata

    # @micropython.native
    def resetPos(self, x: float = 0, y: float = 0, theata: float = 0) -> None:
        with self.drivebase.lock:
            self.x, self.y, self.theata = x, y, theata
        self.drivebase.gyro.reset(theata)

    # @micropython.native
    def getRot(self) -> [float, float]:
        with self.drivebase.lock:
            return self.lm_phi, self.rm_phi
