from pybricks.ev3devices import Motor as _Motor
from pybricks.ev3devices import GyroSensor as _Gyro
from pybricks.parameters import Port, Direction, Stop
from controllers import PIDController
from pybricks.tools import StopWatch


class Motor:
    def __init__(self, Port: Port, Direction: Direction, Kpid=None):
        self.motor = _Motor(Port, Direction)
        if Kpid:
            self.Kp, self.Ki, self.Kd = Kpid
        else:
            self.Kp, self.Ki, self.Kd = 0, 0, 0

    def PIDRunAngle(self, target: int, speed: int, range=(0, 0), Kpid=None):

        """
        This function turns the motor the given angle using a PID controller.

        Parameters: Target-degrees Speed-deg/s
        Range-(-range, +range) Kpid-(number, number, number)

        """

        if Kpid:
            Kp, Ki, Kd = Kpid
        else:
            Kp, Ki, Kd = self.Kp, self.Ki, self.Kd

        PID = PIDController(Kp, Ki, Kd, target)

        angle = self.getAngle()

        while True:
            while (angle < target-range[0] and angle > target+range[1]):

                angle = self.getRawAngle()

                correction = PID.correction(angle)

                if (abs(correction) < speed):
                    correction = 0

                if (target > angle):
                    if (correction > 0):
                        self.run(speed-correction)
                    else:
                        self.run(speed+correction)
                else:
                    if (correction > 0):
                        self.run(-speed+correction)
                    else:
                        self.run(-speed-correction)

            self.stop()

            angle = self.getAngle()
            if (angle >= target-range[0] and angle <= target+range[1]):
                self.stop()
                break

    def PIDRunTarget(self, target: int, speed: int, range=(0, 0), Kpid=None):

        """
        This function turns the motor to the given
        target using a PID controller.

        Parameters: Target-degrees Speed-deg/s
        Range-(-range, +range) Kpid-(number, number, number)

        """

        target = target-self.getAngle()

        self.PIDRunAngle(speed, target, range, Kpid)

    def run(self, speed: int):

        """
        This function runs the motor at the given speed.

        Parameters: Speed-deg/s

        """

        self.motor.run(speed)

    def runAngle(self, target: int, speed: int, stop: Stop, wait=True):

        """
        This function turns the motor to the given angle.

        Parameters: Target-degrees Speed-deg/s
        Stop-StopType Wait-Y/N

        """

        self.motor.run_angle(speed, target, stop, wait)

    def runTarget(self, target: int, speed: int, stop: Stop, wait=True):

        """
        This function turns the motor the given angle.

        Parameters: Target-degrees Speed-deg/s
        Stop-StopType Wait-Y/N

        """

        self.motor.run_target(speed, target, stop, wait)

    def stop(self):

        """
        This function stops the motor and lets it spin freely.
        """

        self.motor.stop()

    def brake(self):

        """
        This function passively brakes the motor.
        """

        self.motor.brake()

    def hold(self):

        """
        This function stops the motor and actively holds it
        at its current angle.
        """

        self.motor.hold()

    def resetAngle(self, angle: int):

        """
        This function resets the motor's angle to the given angle.

        Parameters: Angle-degrees

        """

        self.motor.reset_angle(angle)

    def getAngle(self) -> int:

        """
        This function returns the motor's angle.

        Returns: Angle-degrees(0-360)

        """

        return self.motor.angle() % 360

    def getRawAngle(self) -> int:

        """
        This function returns the motor's angle.

        Returns: Angle-degrees(-inf-inf)

        """

        return self.motor.angle()

    def getSpeed(self) -> int:

        """
        This function returns the motor's speed.

        Returns: Speed-deg/s

        """

        return self.motor.speed()


class Gyro:
    def __init__(self, Port: Port, Direction: Direction):
        self.gyro = _Gyro(Port, Direction)

    def resetAngle(self, angle: int):
        """
        This function resets the gyro's angle to the given angle.
        """

        self.gyro.reset_angle(angle)

    def getAngle(self) -> int:
        """
        This function returns the gyro's angle.

        Returns: Angle-degrees(0-360)

        """

        return self.gyro.angle() % 360

    def getRawAngle(self) -> int:

        """
        This function returns the gyro's angle.

        Returns: Angle-degrees(-inf-inf)

        """

        return self.gyro.angle()

    def getSpeed(self) -> int:

        """
        This function returns the gyro's speed.

        Returns: Speed-deg/s

        """

        return self.gyro.speed()

    def checkOscillation(self, time) -> bool:

        """
        This function checks if the gyro is oscillating.

        Parameters: Time-seconds

        Returns: True/False

        """

        timer = StopWatch()
        pastAngle = self.getRawAngle()
        while (timer.time()/1000 < time):
            angle = self.getRawAngle()
            if (pastAngle != angle):
                return True
            pastAngle = angle
        else:
            return False
