import micropython
from ev3devices import Motor, Gyro, DualGyro


class DriveBase:
    """
    DriveBase class
    Used to control the drivebase.
    Parameters:
        config: object
    """
    def __init__(self, config: object):

        self.lm = Motor(config.drivebase.motor.left)
        self.rm = Motor(config.drivebase.motor.right)
        if not hasattr(config.gyro, 'S2'): self.gyro = Gyro(config.gyro.S1)
        else: self.gyro = DualGyro(config.gyro.S1, config.gyro.S2)

        self._wheelRad = micropython.const(config.wheel.radius)
        self._wheelDiameter = micropython.const(config.wheel.diameter)
        self._wheelCircumference = micropython.const(config.wheel.circumference)

        self._DBM = micropython.const(config.drivebase.DBM)
        self._halfDBM = micropython.const(config.drivebase.halfDBM)

    @micropython.native
    def run_tank(self, Vl: float, Vr: float, Al: float = 0, Ar: float = 0) -> None:
        """
        Run the drivebase at a given speed and acceleration.
        Parameters:
            Vl: float - Left motor speed
            Al: float - Left motor acceleration
            Vr: float - Right motor speed
            Ar: float - Right motor acceleration
        """
        self.lm.run(Vl, Al)
        self.rm.run(Vr, Ar)

    @micropython.native
    def run_tankCM(self, Vl: float, Vr: float, Al: float = 0, Ar: float = 0) -> None:
        """
        Run the drivebase at a given speed and acceleration.
        Parameters:
            Vl: float - Left motor speed
            Al: float - Left motor acceleration
            Vr: float - Right motor speed
            Ar: float - Right motor acceleration
        """
        self.lm.run(self.motorSpeed(Vl), self.motorSpeed(Al))
        self.rm.run(self.motorSpeed(Vr), self.motorSpeed(Ar))

    @micropython.native
    def update(self) -> None:
        """
        PID controller for the motor speeds.
        Update time: 10ms.
        """

        self.lm.update()
        self.rm.update()

    @micropython.native
    def stopUpdate(self) -> None:
        """
        Stops the PID controller.
        """
        self.lm.stopUpdate()
        self.rm.stopUpdate()

    @micropython.native
    def stop(self) -> None:
        """
        Stop the drivebase
        """
        self.lm.stop()
        self.rm.stop()

    @micropython.native
    def motorSpeed(self, V: float) -> float:
        """
        Convert the speed to motor speed.
        CM to Deg
        Parameters:
            V: float - Speed
        Returns:
            speed: float
        """
        return V/self._wheelCircumference*360

    @micropython.native
    def getAngle(self) -> [float, float]:
        """
        Get the angle of the motors of the drivebase.
        Returns:
            [leftAngle, rightAngle]: [float, float]
        """
        return self.lm.getAngle(), self.rm.getAngle()

    @micropython.native
    def getRot(self) -> [float, float]:
        """
        Get the rotation of the motors of the drivebase.
        Returns:
            [leftRot, rightRot]: [float, float]
        """
        return self.lm.getRot(), self.rm.getRot()

    @micropython.native
    def getSpeed(self) -> [float, float]:
        """
        Get the speed of the motors of the drivebase.
        Returns:
            [leftSpeed, rightSpeed]: [float, float]
        """
        return self.lm.getSpeed(), self.rm.getSpeed()

    @micropython.native
    def reset(self) -> None:
        """
        Reset the motors and gyro of the drivebase.
        """
        self.lm.reset(); self.rm.reset()
        self.gyro.reset(0)

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

        dataL = self.lm.analysis(False)
        dataR = self.rm.analysis(False)

        if save:
            with open('/home/robot/Logs/left'+filename, 'w') as f:
                f.write(str(dataL))
            with open('/home/robot/Logs/right'+filename, 'w') as f:
                f.write(str(dataR))

        return dataL, dataR
