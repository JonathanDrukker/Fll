import micropython
from _thread import LockType
from math import sin, cos, radians
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
        """
        Starts the odometry thread.
        """
        self.run = True

        past_lm_pos, past_rm_pos = self.drivebase.getRot()
        theata = self.theata

        while self.run:

            self.lm_phi, self.rm_phi = self.drivebase.getRot()

            Vl = self.lm_phi - past_lm_pos
            Vr = self.rm_phi - past_rm_pos

            V = self.drivebase._wheelCircumference*(Vr + Vl)/2
            theata = radians(self.drivebase.gyro.getProcessedAngle())

            Vx = V*cos(theata)
            Vy = V*sin(theata)

            with self.lock:
                self.x += Vx
                self.y += Vy
                self.theata = theata

            past_lm_pos, past_rm_pos = self.lm_phi, self.rm_phi

    # @micropython.native
    def stop(self) -> None:
        """
        Stops the odometry thread.
        """
        self.run = False

    # @micropython.native
    def getPos2d(self) -> [float, float, float]:
        """
        Returns the current position of the robot.
        Returns:
            x: float
            y: float
            theata: float
        """
        with self.drivebase.lock:
            return self.x, self.y, self.theata

    # @micropython.native
    def resetPos(self, x: float = 0, y: float = 0, theata: float = 0) -> None:
        """
        Resets the position of the robot.
        Parameters:
            x: float
            y: float
            theata: float
        """
        with self.drivebase.lock:
            self.x, self.y, self.theata = x, y, theata
        self.drivebase.gyro.reset(theata)

    # @micropython.native
    def getPhi(self) -> [float, float]:
        """
        Returns the current angle of the robot.
        Phi - the angle of the wheels.
        Returns:
            lm_phi: float
            rm_phi: float
        """
        with self.drivebase.lock:
            return self.lm_phi, self.rm_phi
