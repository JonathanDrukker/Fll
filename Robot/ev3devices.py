import micropython
from os import listdir
from time import time, sleep
from math import copysign
from mytools import thread, sign


motor_ports = {'A': ['0', None], 'B': ['1', None], 'C': ['2', None], 'D': ['3', None]}
sensor_ports = {'1': None, '2': None, '3': None, '4': None}


class PortError(Exception):
    def __init__(self, port):
        self.port = port


class Motor:
    """
    This class is used to control a motor.
    Parameters:
        config: object
        kwargs: dict
    """

    def __init__(self, config: object, **kwargs):

        self._port = micropython.const(config.port)
        self._portProps = motor_ports[self._port]

        if not self.connected():
            raise PortError(self._port)

        self.dir = micropython.const(config.direction)

        self.countF = open("/sys/bus/iio/devices/iio:device1/in_count"+self._portProps[0]+"_raw", 'r')
        self.frequencyF = open("/sys/bus/iio/devices/iio:device1/in_frequency"+self._portProps[0]+"_input", 'r')
        self.dutyCycleF = open("/sys/class/tacho-motor/motor"+self._portProps[1]+"/duty_cycle_sp", 'w')
        self.commandF = open("/sys/class/tacho-motor/motor"+self._portProps[1]+"/command", 'w')

        self.setCommand()

        self.bias = config.bias - self.count() / 2

        self.speed_sp = 0
        self.acc_sp = 0

        self.Kp = micropython.const(config.p)
        self.Ks, self.Kv, self.Ka = micropython.const(config.ff)

        self._lastFrequency = 0

        for key, value in kwargs.items():
            setattr(self, key, value)

    @micropython.native
    def run(self, speed: float = 0, acc: float = 0) -> None:
        """
        Runs the motor at a given speed and acceleration.
        Uses the ff values to calculate the duty cycle.
        Needed to run Motor.update() to work.
        Parameters:
            speed: float
            acc: float
        """
        self.speed_sp = speed
        self.acc_sp = acc

    @micropython.native
    @thread
    def update(self) -> None:
        """
        FF and P controller for the motor speed.
        Runs as thread.
        Update time: 10ms.
        """

        self.runUpdate = True
        while self.runUpdate:

            speed = self.speed_sp + self.Kp*(self.speed_sp - self.getSpeed())
            self.dutyCycle(self.Ks*sign(speed) + self.Kv*speed + self.Ka*self.acc_sp)

            sleep(0.01)

    @micropython.native
    def stopUpdate(self) -> None:
        """
        Stops the update thread.
        """
        self.runUpdate = False

    @micropython.native
    def runImmediate(self, speed: float = 0, acc: float = 0) -> None:
        """
        Runs the motor at a given speed and acceleration.
        Uses P and FF controllers.
        Runs IMMEDIATELY.
        Parameters:
            speed: float
            acc: float
        """
        speed += self.Kp*(speed - self.getSpeed())
        self.dutyCycle(self.Ks*sign(speed) + self.Kv*speed + self.Ka*acc)

    @micropython.native
    def stop(self) -> None:
        """
        Stops the motor and sets speed_sp and acc_sp to 0.
        """
        self.speed_sp = 0
        self.acc_sp = 0
        self.dutyCycle(0)

    @micropython.native
    def getAngle(self) -> float:
        """
        Returns the angle of the motor in degrees.
        Returns:
            angle: float
        """
        return self.count() / 2 + self.bias

    @micropython.native
    def getRot(self) -> float:
        """
        Returns the rotation of the motor in rotations(1Rot = 360Deg).
        Returns:
            rotation: float
        """
        return self.getAngle() / 360

    @micropython.native
    def getSpeed(self) -> float:
        """
        Returns the speed of the motor in degrees per second.
        Returns:
            speed: float
        """
        return self.frequency() / 2

    @micropython.native
    def reset(self, bias: float = 0) -> None:
        """
        Resets the angle of the motor.
        Parameters:
            bias: float
        """
        self.bias = bias - self.count() / 2

    @micropython.native
    def count(self) -> int:
        """
        Returns the count of the motor.
        Returns:
            count: int
        """
        self.countF.seek(0)
        return int(self.countF.read()) * self.dir

    @micropython.native
    def frequency(self) -> int:
        """
        Returns the frequency of the motor.
        Returns:
            frequency: int
        """
        self.frequencyF.seek(0)
        data = self.frequencyF.read()
        if data.strip().lstrip("-").isdigit():
            self._lastFrequency = data
        return int(self._lastFrequency) * self.dir

    @micropython.native
    def dutyCycle(self, dutyCycle: int) -> None:
        """
        Sets the duty cycle of the motor.
        Duty cycle is a value between -100 and 100 but
            function will limit it to that range.
        Parameters:
            dutyCycle: int
        """
        self.dutyCycleF.seek(0)
        if abs(dutyCycle) <= 100:
            self.dutyCycleF.write(str(int(dutyCycle)))
        else:
            self.dutyCycleF.write(str(int(copysign(100, dutyCycle))))
        self.dutyCycleF.flush()

    @micropython.native
    def setCommand(self, command: str = "run-direct"):
        """
        Sets the command of the motor.
        Parameters:
            command: str
        """
        self.commandF.seek(0)
        self.commandF.write(command)
        self.commandF.flush()

    @micropython.native
    def analysis(self, save: bool = True, filename: str = 'analysis.log') -> tuple:
        """
        Returns the analysis of the motor.
        Parameters:
            save: bool
            filename: str
        Returns:
            data: list
        """

        data = []

        self.dutyCycle(-100)
        sleep(3)

        st = time()
        for duty in range(-100, 101):

            self.dutyCycle(duty)

            loopT = time()
            while time() - loopT < 0.5:
                data.append([time() - st, duty, self.getSpeed()])

        self.dutyCycle(0)

        if save:
            with open("/home/robot/Logs/"+filename, 'w') as f:
                f.write(str(data))

        return data

    @micropython.native
    def connected(self) -> bool:
        """
        Returns if the motor is connected.
        Returns:
            connected: bool
        """
        try:
            with open("/sys/class/tacho-motor/motor"+self._portProps[1]+"/address", 'r'):
                return True
        except Exception:
            return False


class Gyro:
    """
    This class is used to control a gyro sensor.
    Parameters:
        config: object
        kwargs: dict
    """

    def __init__(self, config: object, **kwargs):

        self._port = micropython.const(config.port)
        self._portF = sensor_ports[self._port]

        if not self.connected():
            raise PortError(self._port)

        self.dir = micropython.const(config.direction)

        self.valueF = open("/sys/class/lego-sensor/sensor"+self._portF+"/value0", 'r')
        self.modeF = open("/sys/class/lego-sensor/sensor"+self._portF+"/mode", 'w')

        self.setMode()

        self.bias = config.bias - self.read()

        for key, value in kwargs.items():
            setattr(self, key, value)

    @micropython.native
    def getAngle(self) -> float:
        """
        Returns the angle of the gyro in degrees.
        Returns:
            angle: float
        """
        return self.read() + self.bias

    def getProcessedAngle(self) -> float:
        """
        Returns the angle of the gyro in degrees.
        Returns:
            angle: float - [0, 360]
        """
        angle = self.getAngle() % 360
        if angle < 0:
            angle += 360
        return angle

    @micropython.native
    def getSpeed(self) -> float:
        """
        Returns the speed of the gyro in degrees per second.
        Returns:
            speed: float
        """
        self.setMode("GYRO-RATE")
        val = self.read()
        self.setMode("GYRO-ANG")
        return val

    @micropython.native
    def reset(self, angle: float = 0) -> None:
        """
        Resets the angle of the gyro.
        Parameters:
            angle: float
        """
        self.bias = angle - self.read()

    @micropython.native
    def calibrate(self, angle: float = 0) -> None:
        """
        Calibrates and resets the gyro.
        Used to stop drift.
        GYRO IS NOT ALLOWED TO MOVE WHILE CALIBRATING.
        Calibration time: 2 seconds.
        """
        self.setMode("GYRO-CAL")
        sleep(2)
        self.setMode("GYRO-ANG")
        self.reset(angle)

    @micropython.native
    def read(self) -> int:
        """
        Returns the value currently reading.
        Returns:
            value: int
        """
        self.valueF.seek(0)
        return int(self.valueF.read()) * self.dir

    @micropython.native
    def setMode(self, mode: str = "GYRO-ANG") -> None:
        """
        Sets the mode of the gyro.
        Parameters:
            mode: str
        """
        self.modeF.seek(0)
        self.modeF.write(mode)
        self.modeF.flush()

    @micropython.native
    def connected(self) -> bool:
        """
        Returns if the motor is connected.
        Returns:
            connected: bool
        """
        try:
            with open("/sys/class/lego-sensor/sensor"+self._portF+"/address", 'r'):
                return True
        except Exception:
            return False


class DualGyro(Gyro):
    """
    This class is used to control two gyro sensors simultaneously.
    Parameters:
        config1: object
        config2: object
        kwargs: dict
    """

    def __init__(self, config1: dict, config2: dict, **kwargs):
        self.gyro1 = Gyro(config1)
        self.gyro2 = Gyro(config2)

        for key, value in kwargs.items():
            setattr(self.gyro1, key, value)
            setattr(self.gyro2, key, value)

    @micropython.native
    def getAngle(self) -> float:
        """
        Returns the angle of the gyro in degrees.
        Returns:
            angle: float
        """
        return (self.gyro1.getAngle() + self.gyro2.getAngle()) / 2

    @micropython.native
    def getProcessedAngle(self) -> float:
        """
        Returnsx the angle of the gyro in degrees.
        Returns:
            angle: float - [0, 360]
        """
        theata = self.getAngle() % 360
        if theata < 0:
            theata += 360
        return theata

    @micropython.native
    def getSpeed(self) -> float:
        """
        Returns the speed of the gyro in degrees per second.
        Returns:
            speed: float
        """
        return (self.gyro1.getSpeed() + self.gyro2.getSpeed()) / 2

    @micropython.native
    def reset(self, angle: int = 0) -> None:
        """
        Resets the angle of the gyro.
        Parameters:
            angle: float
        """
        self.gyro1.reset(angle)
        self.gyro2.reset(angle)

    @micropython.native
    def calibrate(self, angle: int = 0) -> None:
        """
        Calibrates the gyro.
        Used to stop drift.
        GYROS IS NOT ALLOWED TO MOVE WHILE CALIBRATING.
        Calibration time: 4 seconds.
        """
        self.gyro1.calibrate(angle)
        self.gyro2.calibrate(angle)

    @micropython.native
    def connected(self) -> [bool, bool]:
        """
        Returns if the motor is connected.
        Returns:
            connected: bool
        """
        return self.gyro1.connected(), self.gyro2.connected()


class LightSensor:
    """
    This class is used to control a light sensor.
    Parameters:
        config: object
        kwargs: dict
    """

    def __init__(self, config: object, **kwargs):

        self._port = micropython.const(config.port)
        self._portF = sensor_ports[config.port]

        if not self.connected():
            raise PortError(self._port)

        self.valueF = open("/sys/class/lego-sensor/sensor"+self._portF+"/value0", 'r')
        self.modeF = open("/sys/class/lego-sensor/sensor"+self._portF+"/mode", 'w')

        self.setMode()

        for key, value in kwargs.items():
            setattr(self, key, value)

    @micropython.native
    def getReflect(self) -> int:
        """
        Returns the reflectivity of the light sensor.
        Returns:
            reflectivity: int
        """
        return self.read()

    @micropython.native
    def getColor(self) -> int:
        """
        Returns the color of the light sensor.
        Returns:
            color: int
        """
        self.setMode('COL-COLOR')
        val = self.read()
        self.setMode("COL-REFLECT")
        return val

    @micropython.native
    def read(self) -> int:
        """
        Returns the value currently reading.
        Returns:
            value: int
        """
        self.valueF.seek(0)
        return int(self.valueF.read())

    @micropython.native
    def setMode(self, mode: str = "COL-REFLECT") -> None:
        """
        Sets the mode of the light sensor.
        Parameters:
            mode: str
        """
        self.modeF.seek(0)
        self.modeF.write(mode)
        self.modeF.flush()

    @micropython.native
    def connected(self) -> bool:
        """
        Returns if the motor is connected.
        Returns:
            connected: bool
        """
        try:
            with open("/sys/class/lego-sensor/sensor"+self._portF+"/address", 'r'):
                return True
        except Exception:
            return False


@micropython.native
def MotorPorts() -> None:
    """
    Updates dict of motor ports.
    """
    for dir in listdir("/sys/class/tacho-motor"):
        with open("/sys/class/tacho-motor/"+dir+"/address", 'r') as f:
            addr = f.read()
        motor_ports[addr[13]][1] = dir[5:]


@micropython.native
def SensorPorts() -> None:
    """
    Updates dict of sensor ports.
    """
    for dir in listdir("/sys/class/lego-sensor"):
        with open("/sys/class/lego-sensor/"+dir+"/address", 'r') as f:
            addr = f.read()
        sensor_ports[addr[12]] = dir[6:]


@micropython.native
def startup() -> None:
    """
    Updates dicts of ports.
    """
    MotorPorts()
    SensorPorts()


startup()
