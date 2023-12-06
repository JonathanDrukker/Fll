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

    def __init__(self, light: object, drivebase: object):

        self.ll = LightSensor(light.SL)
        self.rl = LightSensor(light.SR)

        self.drivebase = drivebase

    @micropython.native
    def Drive(self, speed: float, dist: float, target: float,
              Kp: float, Ki: float, Kd: float, timeout: float = 60) -> None:
        """
        Used to drive with the drivebase using the gyro and PID.
        Parameters:
            speed: float - Speed
            dist: float - Distance
            target: float - Target
            Kp: float - Proportional gain
            Ki: float - Integral gain
            Kd: float - Derivative gain
            timeout: float - Max time
        """
        PID = PIDController(Kp, Ki, Kd, target)

        startAngle = mean(self.drivebase.getAngle())
        st = time()

        while (mean(self.drivebase.getAngle()) - startAngle < dist / self.drivebase._wheelCircumference and
               time() - st < timeout):

            correction = PID.correction(self.drivebase.gyro.angle())

            self.drivebase.run_tank(speed+correction, speed-correction)

    @micropython.native
    def Turn(self, speed: float, angle: float, target: float,
             Kp: float, Ki: float, Kd: float, timeout: float = 60) -> None:
        """
        Used to turn with the drivebase using the gyro and PID.
        Parameters:
            speed: float - Speed
            angle: float - Angle
            target: float - Target
            Kp: float - Proportional gain
            Ki: float - Integral gain
            Kd: float - Derivative gain
            timeout: float - Max time
        """
        PID = PIDController(Kp, Ki, Kd, target)

        st = time()

        while (self.drivebase.gyro.angle() < angle and
               time() - st < timeout):

            correction = PID.correction(self.drivebase.gyro.angle())

            self.drivebase.run_tank(speed+correction, -speed-correction)

    @micropython.native
    def LineFollow(self, speed: float, dist: float, rfl: int, side: int,
                   Kp: float, Ki: float, Kd: float, timeout: float = 60) -> None:
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
            timeout: float - Max time
        """

        PID = PIDController(Kp, Ki, Kd, rfl)
        cl = self.ll if side == -1 else self.rl

        startAngle = mean(self.drivebase.getAngle())

        st = time()

        while (mean(self.drivebase.getAngle()) - startAngle <
               dist / self.drivebase._wheelCircumference and
               time() - st < timeout):

            corr = PID.correction(cl.getReflectedLight())
            self.drivebase.run_tank(speed + corr, speed - corr)

    @micropython.native
    def Box(self, speed: int, rfl: int, Kp: float, Ki: float, Kd: float, timeout: float = 60) -> None:
        """
        Used to follow a line with the drivebase using light sensors and the gyro.
        Parameters:
            speed: int - Speed
            rfl: int - Reflectance value
            side: int - Side
            timeout: float - Max time
        """
        PID = PIDController(Kp, Ki, Kd, rfl)

        st = time()

        lRFL, rRFL = self.ll.getReflect(), self.rl.getReflect()

        while (lRFL != rfl and rRFL != rfl and time() - st < timeout):

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
