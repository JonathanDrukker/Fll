from pybricks.ev3devices import Motor as _Motor
from pybricks.ev3devices import GyroSensor as _Gyro
from pybricks.ev3devices import ColorSensor as _ColorSensor
from pybricks.parameters import Port, Direction, Stop, Color
from controllers import PIDController
from time import time
from mymath import mean
import _thread


class Motor:

    lock = _thread.allocate_lock()
    runFunc = True

    def __init__(self, Port: Port, Direction: Direction, Kpid=None):
        self.motor = _Motor(Port, Direction)
        if Kpid:
            self.Kp, self.Ki, self.Kd = Kpid
        else:
            self.Kp, self.Ki, self.Kd = 0, 0, 0

    def PIDRunAngle(self, target: int, speed: int, range=(0, 0),
                    Kpid=None, wait=True):
        """
        This function turns the motor the given angle using a PID controller.

        Parameters: Target-degrees Speed-deg/s
        Range-(-range, +range) Kpid-(number, number, number), wait-True/False

        """

        def run():

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

        if wait:
            run()
        else:
            _thread.start_new_thread(run, ())

    def PIDRunTarget(self, target: int, speed: int, range=(0, 0),
                     Kpid=None, wait=True):
        """
        This function turns the motor to the given
        target using a PID controller.

        Parameters: Target-degrees Speed-deg/s
        Range-(-range, +range) Kpid-(number, number, number)

        """

        target = target-self.getAngle()

        self.PIDRunAngle(speed, target, range, Kpid, wait)

    def run(self, speed: int):
        """
        This function runs the motor at the given speed.

        Parameters: Speed-deg/s

        """

        self.motor.run(speed)

    def PIDRun(self, target: int, Kpid=None):
        """
        This function runs the motor at the given speed using a PID controller.

        Parameters: Target-deg/s Kpid-(number, number, number)

        """

        def run():

            if Kpid:
                Kp, Ki, Kd = Kpid
            else:
                Kp, Ki, Kd = self.Kp, self.Ki, self.Kd

            PID = PIDController(Kp, Ki, Kd, target)

            pastAngle = self.getRawAngle()
            pastTime = time()

            while self.runFunc:
                angle = self.getRawAngle()
                cTime = time()

                vel = (angle-pastAngle)/(cTime-pastTime)
                correction = PID.correction(vel)

                if self.runFunc:
                    self.run(target+correction)

            self.runFunc = True

        _thread.start_new_thread(run, ())

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

        self.runFunc = False
        with self.lock:
            self.motor.stop()

    def brake(self):
        """
        This function passively brakes the motor.
        """

        self.runFunc = False
        with self.lock:
            self.motor.brake()

    def hold(self):
        """
        This function stops the motor and actively holds it
        at its current angle.
        """

        self.runFunc = False
        with self.lock:
            self.motor.hold()

    def trueStop(self):
        """
        This function stops then brakes it - the motor and actively holds
        itself at its current angle.
        """

        self.stop()
        self.hold()

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

    def getRawAngle(self) -> int:
        """
        This function returns the gyro's angle.

        Returns: Angle-degrees(-inf-inf)

        """

        return self.gyro.angle()

    def getAngle(self) -> int:
        """
        This function returns the gyro's angle.

        Returns: Angle-degrees(0-360)

        """

        return self.getRawAngle() % 360

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

        startTime = time()
        pastAngle = self.getRawAngle()
        while (time()-startTime < time):
            angle = self.getRawAngle()
            if (pastAngle != angle):
                return True
            pastAngle = angle
        else:
            return False


class DualGyro(Gyro):
    def __init__(self, Port1: Port, Direction1: Direction,
                 Port2: Port, Direction2: Direction):
        self.gyro1 = _Gyro(Port1, Direction1)
        self.gyro2 = _Gyro(Port2, Direction2)

    def resetAngle(self, angle):
        """
        This function resets the gyro's angle to the given angle.
        """

        self.gyro1.reset_angle(angle)
        self.gyro2.reset_angle(angle)

    def getRawAngle(self):
        """
        This function returns the gyro's angle.

        Returns: Angle-degrees(-inf-inf)

        """

        return mean(self.gyro1.angle(), self.gyro2.angle())

    def getSpeed(self):
        """
        This function returns the gyro's speed.

        Returns: Speed-deg/s

        """

        return mean(self.gyro1.speed(), self.gyro2.speed())


class LightSensor:
    def __init__(self, Port: Port, MAX: float, MIN: float):
        self.sensor = _ColorSensor(Port)
        self.MAX = MAX
        self.MIN = MIN

    def reflection(self) -> float:
        """
        This function returns the reflection measerment.

        Returns: Reflection-percent
        """

        return self.sensor.reflection()

    def ambient(self) -> float:
        """
        This function returns the ambient mesearment.

        Returns: Ambient-percent
        """
        return self.sensor.ambient()

    def color(self) -> Color:
        """
        This function return the color measerment.

        Returns: Color-Color
        """

        return self.sensor.color()

    def RGB(self) -> tuple:
        """
        This function return the color measerment in RGB values.

        Returns: RGB-(Percent, Percent, Percent)
        """

        return self.sensor.rgb()
