import micropython
from ev3devices import Motor as _Motor
from controllers import PIDController
from math import copysign
from time import time, sleep
from mytools import thread


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

    # @micropython.native
    def TurnABS(self, angle: float, Kpid: [float, float, float] = None, speed: float = 0,
                timeout: float = 60, wait: bool = True):
        """
        Turn the motor to a specific angle.
        Parameters:
            angle: float
            Kpid: [float, float, float]
            speed: float
            timeout: float
            wait: bool
        """
        # @micropython.native
        def main(angle: float, Kp: float = None, Ki: float = None, Kd: float = None, speed: float = 0,
                 timeout: float = 60):
            if Kpid is None:
                Kp, Ki, Kd = self.posKpid
            PID = PIDController(Kp, Ki, Kd, angle)

            st = time()

            error = 1
            while error != 0 and time() - st < timeout:
                error = angle - self.getAngle()
                self.runImmediate(PID.correction(None, error) + copysign(speed, error))

            self.stop()

        if wait:
            main(angle, Kpid, speed, timeout)
        else:
            thread(main)(angle, Kpid, speed, timeout)

    # @micropython.native
    def Turn(self, angle: float, Kpid: [float, float, float] = None, speed: float = 0,
             timeout: float = 60, wait: bool = True):
        """
        Turn the motor by a specific angle.
        Parameters:
            angle: float
            Kpid: [float, float, float]
            speed: float
            timeout: float
            wait: bool
        """
        self.TurnABS(self.getAngle() + angle, Kpid, speed, timeout, wait)

    def RunTime(self, speed: float, time: float, wait: bool = True):
        """
        Run the motor for a specific time.
        Parameters:
            speed: float
            time: float
            wait: bool
        """
        # @micropython.native
        def main(speed: float, time: float):
            self.runImmediate(speed)
            sleep(time)
            self.stop()

        if wait:
            main(speed, time)
        else:
            thread(main)(speed, time)
