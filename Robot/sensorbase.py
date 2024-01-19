import micropython
from time import time, sleep
from ev3devices import LightSensor
from controllers import PIDController
from mytools import mean, between


class Sensorbase:
    """
    Sensorbase class
    Used to control the sensors and the drivebase with speical functions.
    Parameters:
        lLight: str
        rLight: str
        drivebase: object
    """

    def __init__(self, config: object, light: object, drivebase: object):

        self.config = config

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
    def Turn(self, target: float, Kp: float, Ki: float, Kd: float, range: float = 1, timeout: float = 60):
        """
        Used to turn with the drivebase using the gyro and PID.
        Parameters:
            target: float - Target [0-360]
            Kp: float - Proportional gain
            Ki: float - Integral gain
            Kd: float - Derivative gain
            error: float - Error
            timeout: float - Max time
        """

        PID = PIDController(Kp, Ki, Kd, target)

        st = time()

        error = target - self.drivebase.gyro.getProcessedAngle()
        if error < -180:
            error += 360

        while (not between(error, 0, range) and
               time() - st < timeout):

            while (not between(error, 0, range) and
                   time() - st < timeout):

                error = target - self.drivebase.gyro.getProcessedAngle()
                if error < -180:
                    error += 360

                correction = PID.correction(None, error)

                self.drivebase.run_tank(-correction, correction)

            self.drivebase.stop()
            sleep(0.1)

            error = target - self.drivebase.gyro.getProcessedAngle()
            if error < -180:
                error += 360

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
    def Box(self, rfl: int, speed: int, Kp: float = None, Ki: float = None, Kd: float = None, timeout: float = 60, range: int = 10) -> None:
        """
        Used to follow a line with the drivebase using light sensors and the gyro.
        Parameters:
            speed: int - Speed
            rfl: int - Reflectance value
            side: int - Side
            timeout: float - Max time
        """

        st = time()

        lRFL, rRFL = self.ll.getReflect(), self.rl.getReflect()

        while not between(lRFL, rfl, range) and not between(rRFL, rfl, range) and time() - st < timeout:

            while not between(lRFL, rfl, range) and not between(rRFL, rfl, range) and time() - st < timeout:

                self.drivebase.run_tank(speed, speed)

                lRFL, rRFL = self.ll.getReflect(), self.rl.getReflect()
                print(lRFL, rRFL)

            sleep(0.1)

        if between(lRFL, rfl, range):

            self.drivebase.run_tank(0, speed)
            self.drivebase.lm.stop()

            while not between(rRFL, rfl, range) and time() - st < timeout:
                while not between(rRFL, rfl, range) and time() - st < timeout:

                    self.drivebase.run_tank(0, speed)
                    lRFL, rRFL = self.ll.getReflect(), self.rl.getReflect()
                    print(lRFL, rRFL)

                sleep(0.1)
                self.drivebase.stop()

        else:

            self.drivebase.run_tank(speed, 0)
            self.drivebase.rm.stop()

            while not between(lRFL, rfl, range) and time() - st < timeout:
                while not between(lRFL, rfl, range) and time() - st < timeout:

                    self.drivebase.run_tank(speed, 0)
                    lRFL, rRFL = self.ll.getReflect(), self.rl.getReflect()
                    print(lRFL, rRFL)

                sleep(0.1)
                self.drivebase.stop()
