import micropython
from _thread import LockType
from math import sin, cos, radians, degrees
from mytools import thread


class DiffrentialDriveOdometry:
    """
    This class is used to calculate the odometry of a diffrential drive robot.
    Odometry is the use of data from motion sensors to estimate change in position over time.
    Parameters:
        drivebase: DriveBase object
        lock: LockType object
        x: float
        y: float
        theata: float
    """
    def __init__(self, drivebase: object, lock: LockType, x: float = 0, y: float = 0, theata: float = 0):

        self.lock = lock

        self.drivebase = drivebase

        self.x, self.y = x, y
        self.theata = theata

        self.run = False

    @micropython.native
    @thread
    def start(self) -> None:
        """
        Starts the odometry thread.
        """
        self.run = True

        past_lm_pos, past_rm_pos = self.drivebase.getRot()

        while self.run:

            lm_phi, rm_phi = self.drivebase.getRot()

            Vl = lm_phi - past_lm_pos
            Vr = rm_phi - past_rm_pos

            V = self.drivebase._wheelCircumference*(Vr + Vl)/2
            theata = radians(self.drivebase.gyro.getProcessedAngle())

            Vx = V*cos(theata)
            Vy = V*sin(theata)

            with self.lock:
                self.x += Vx
                self.y += Vy
                self.theata = theata

            past_lm_pos, past_rm_pos = lm_phi, rm_phi

    @micropython.native
    def stop(self) -> None:
        """
        Stops the odometry thread.
        """
        self.run = False

    @micropython.native
    def getPos2d(self) -> [float, float, float]:
        """
        Returns the current position of the robot.
        Returns:
            x: float
            y: float
            theata: float
        """
        with self.lock:
            return self.x, self.y, self.theata

    @micropython.native
    def resetPos(self, x: float = 0, y: float = 0, theata: float = 0) -> None:
        """
        Resets the position of the robot.
        Parameters:
            x: float
            y: float
            theata: float
        """
        self.drivebase.gyro.reset(degrees(theata))
        with self.lock:
            self.x, self.y, self.theata = x, y, theata
