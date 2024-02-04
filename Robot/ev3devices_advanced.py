import micropython
from ev3devices import Motor as _Motor
from controllers import PIDController
from math import copysign
from time import time, sleep
from mytools import thread, between


class Motor(_Motor):
    """
    Motor class
    Used to control the motor more easily, intuitionally and more functionally.
    Parameters:
        Port: str
        Direction: int
        bias: float
        speedKpid: [float, float, float]
        posKpid: [float, float, float]
    """

    def __init__(self, config, **kwargs):

        super().__init__(config)

        self.speedKpid = micropython.const(config.speedKpid)
        self.posKpid = micropython.const(config.posKpid)

    @micropython.native
    def TurnABS(self, angle: float, Kpid: [float, float, float] = None, speed: float = 0,
                timeout: float = 60, wait: bool = True, range: float = 1):
        """
        Turn the motor to a specific angle.
        Parameters:
            angle: float
            Kpid: [float, float, float]
            speed: float
            timeout: float
            wait: bool
            range: float
        """
        @micropython.native
        def main(self, angle: float, Kpid: [float, float, float] = None, speed: float = 0,
                 timeout: float = 60, range: float = 1):
            if Kpid is None:
                Kp, Ki, Kd = self.posKpid
            else:
                Kp, Ki, Kd = Kpid
            PID = PIDController(Kp, Ki, Kd, angle)

            st = time()

            error = angle - self.getAngle()
            while not between(error, 0, range) and time() - st < timeout:

                while not between(error, 0, range) and time() - st < timeout:
                    error = angle - self.getAngle()
                    self.runImmediate(PID.correction(None, error) + copysign(speed, error))

                self.stop()

                sleep(0.1)
                error = angle - self.getAngle()

        if wait:
            main(self, angle, Kpid, speed, timeout, range)
        else:
            thread(main)(self, angle, Kpid, speed, timeout, range)

    @micropython.native
    def Turn(self, angle: float, Kpid: [float, float, float] = None, speed: float = 0,
             timeout: float = 60, wait: bool = True, range: float = 1):
        """
        Turn the motor by a specific angle.
        Parameters:
            angle: float
            Kpid: [float, float, float]
            speed: float
            timeout: float
            wait: bool
            range: float
        """
        self.TurnABS(self.getAngle() + angle, Kpid, speed, timeout, wait, range)

    def RunTime(self, speed: float, time: float, wait: bool = True):
        """
        Run the motor for a specific time.
        Parameters:
            speed: float
            time: float
            wait: bool
        """
        @micropython.native
        def main(speed: float, time: float):
            self.runImmediate(speed)
            self.runImmediate(speed)
            sleep(time)
            self.stop()

        if wait:
            main(speed, time)
        else:
            thread(main)(speed, time)
