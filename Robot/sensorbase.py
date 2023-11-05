import micropython
from time import time
from ev3devices import LightSensor
from controllers import PIDController
from mytools import mean


class Sensorbase:
    """
    Sensorbase class
    Used to control the sensors and the drivebase with speical functions.
    Parameters:
        lLight: str
        rLight: str
        drivebase: object
    """
    def __init__(self, lLight: str, rLight: str, drivebase: object):

        self.ll = LightSensor(lLight)
        self.rl = LightSensor(rLight)

        self.drivebase = drivebase

    # @micropython.native
    def PIDDrive(self, speed: float, dist: float, target: float,
                 Kp: float, Ki: float, Kd: float, maxTime: float = 60) -> None:
        """
        Used to drive with the drivebase using the gyro and PID.
        Parameters:
            speed: float - Speed
            dist: float - Distance
            target: float - Target
            Kp: float - Proportional gain
            Ki: float - Integral gain
            Kd: float - Derivative gain
            maxTime: float - Max time
        """
        PID = PIDController(Kp, Ki, Kd, target)

        startAngle = mean(self.drivebase.getAngle())
        st = time()

        while (mean(self.drivebase.getAngle()) - startAngle < dist / self.drivebase._wheelCircumference and
               time() - st < maxTime):

            correction = PID.correction(self.drivebase.gyro.angle())

            self.drivebase.run_tank(speed+correction, speed-correction)

    # @micropython.native
    def PIDTurn(self, speed: float, angle: float, target: float,
                Kp: float, Ki: float, Kd: float, maxTime: float = 60) -> None:
        """
        Used to turn with the drivebase using the gyro and PID.
        Parameters:
            speed: float - Speed
            angle: float - Angle
            target: float - Target
            Kp: float - Proportional gain
            Ki: float - Integral gain
            Kd: float - Derivative gain
            maxTime: float - Max time
        """
        PID = PIDController(Kp, Ki, Kd, target)

        st = time()

        while (self.drivebase.gyro.angle() < angle and
               time() - st < maxTime):

            correction = PID.correction(self.drivebase.gyro.angle())

            self.drivebase.run_tank(speed+correction, -speed-correction)

    # @micropython.native
    def PIDLineFollow(self, speed: float, dist: float, rfl: int, side: int,
                      Kp: float, Ki: float, Kd: float, maxTime: float = 60) -> None:
        """
        Used to follow a line with the drivebase.
        Parameters:
            speed: float - Speed
            dist: float - Distance
            rfl: int - Reflectance value
            side: int - Side
            Kp: float - Proportional gain
            Ki: float - Integral gain
            Kd: float - Derivative gain
            maxTime: float - Max time
        """

        PID = PIDController(Kp, Ki, Kd, rfl)
        cl = self.ll if side == -1 else self.rl

        startAngle = mean(self.drivebase.getAngle())

        st = time()

        while (mean(self.drivebase.getAngle()) - startAngle <
               dist / self.drivebase._wheelCircumference and
               time() - st < maxTime):

            corr = PID.correction(cl.getReflectedLight())
            self.drivebase.run_tank(speed + corr, speed - corr)

    # @micropython.native
    def PIDBox(self, speed: int, rfl: int, Kp: float, Ki: float, Kd: float, maxTime: float = 60) -> None:
        """
        Used to follow a line with the drivebase using light sensors and the gyro.
        Parameters:
            speed: int - Speed
            rfl: int - Reflectance value
            side: int - Side
            maxTime: float - Max time
        """
        PID = PIDController(Kp, Ki, Kd, rfl)

        st = time()

        lRFL, rRFL = self.ll.getReflect(), self.rl.getReflect()

        while (lRFL != rfl and rRFL != rfl and time() - st < maxTime):

            correction = PID.correction(lRFL - rRFL)  # check if this is correct

            avgRFL = mean(lRFL, rRFL)
            if avgRFL == 0:
                self.drivebase.run_tank(correction, -correction)
            elif avgRFL > rfl:
                self.drivebase.run_tank(speed+correction, speed-correction)
            else:
                self.drivebase.run_tank(-speed+correction, -speed-correction)

            lRFL, rRFL = self.ll.getReflect(), self.rl.getReflect()

        self.drivebase.stop()
