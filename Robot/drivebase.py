import micropython
from _thread import LockType
from ev3devices import Motor, Gyro, DualGyro


class DriveBase:
    """
    DriveBase class
    Used to control the drivebase.
    Parameters:
        config: object
        lock: LockType
    """
    def __init__(self, config: object, lock: LockType):

        self.lock = lock

        self.lm = Motor(config.drivebase.motor.left)
        self.rm = Motor(config.drivebase.motor.right)
        if hasattr(config, "gyro2"): DualGyro(config.gyro1, config.gyro2)
        else: self.gyro = Gyro(config.gyro)

        self._wheelRad = micropython.const(config.wheel.radius)
        self._wheelDiameter = micropython.const(config.wheel.diameter)
        self._wheelCircumference = micropython.const(config.wheel.circumference)

        self._DBM = micropython.const(config.drivebase.DBM)
        self._halfDBM = micropython.const(config.drivebase.halfDBM)

    # @micropython.native
    def run_tank(self, Vl, Al, Vr, Ar) -> None:
        """
        Run the drivebase at a given speed and acceleration.
        Parameters:
            Vl: float - Left motor speed
            Al: float - Left motor acceleration
            Vr: float - Right motor speed
            Ar: float - Right motor acceleration
        """
        self.lm.run(self.motorSpeed(Vl))
        self.rm.run(self.motorSpeed(Vr))

    # @micropython.native
    def stop(self) -> None:
        """
        Stop the drivebase
        """
        self.lm.stop()
        self.rm.stop()

    # @micropython.native
    def motorSpeed(self, V: float) -> float:
        """
        Convert the speed to motor speed.
        Parameters:
            V: float - Speed
        Returns:
            speed: float
        """
        return V/self._wheelCircumference*360

    # @micropython.native
    def getAngle(self) -> [float, float]:
        """
        Get the angle of the motors of the drivebase.
        Returns:
            [leftAngle, rightAngle]: [float, float]
        """
        return self.lm.getAngle(), self.rm.getAngle()

    # @micropython.native
    def getRot(self) -> [float, float]:
        """
        Get the rotation of the motors of the drivebase.
        Returns:
            [leftRot, rightRot]: [float, float]
        """
        return self.lm.getRot(), self.rm.getRot()

    # @micropython.native
    def getSpeed(self) -> [float, float]:
        """
        Get the speed of the motors of the drivebase.
        Returns:
            [leftSpeed, rightSpeed]: [float, float]
        """
        return self.lm.getSpeed(), self.rm.getSpeed()

    # @micropython.native
    def reset(self) -> None:
        """
        Reset the motors of the drivebase.
        """
        self.lm.reset(); self.rm.reset()

    def analysis(self, save: bool = True, filename: str = 'Analysis.log') -> tuple:
        """
        Analyze the motors of the drivebase.
        Parameters:
            save: bool - Save the analysis to a file
            filename: str - Filename
        Returns:
            [leftDuty, leftSpeed, leftAcceleration]: [list, list, list]
            [rightDuty, rightSpeed, rightAcceleration]: [list, list, list]
        """

        dutyL, speedL, accL = self.lm.analysis(False)
        dutyR, speedR, accR = self.rm.analysis(False)

        if save:
            with open('left'+filename, 'w') as f:
                f.write(str({'duty': dutyL, 'velocity': speedL, 'acceleration': accL}))
            with open('right'+filename, 'w') as f:
                f.write(str({'duty': dutyR, 'velocity': speedR, 'acceleration': accR}))

        return [dutyL, speedL, accL], [dutyR, speedR, accR]
