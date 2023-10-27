import micropython
from os import listdir
from time import time, sleep
from math import copysign


motor_ports = {'A': ['0', None], 'B': ['1', None], 'C': ['2', None], 'D': ['3', None]}
sensor_ports = {'1': None, '2': None, '3': None, '4': None}


class Motor:
    def __init__(self, config: object, **kwargs):

        self._port = micropython.const(config.port)
        self._portProps = motor_ports[self._port]
        self.dir = config.direction

        self.countF = open("/sys/bus/iio/devices/iio:device1/in_count"+self._portProps[0]+"_raw", 'r')
        self.frequencyF = open("/sys/bus/iio/devices/iio:device1/in_frequency"+self._portProps[0]+"_input", 'r')
        self.dutyCycleF = open("/sys/class/tacho-motor/motor"+self._portProps[1]+"/duty_cycle_sp", 'w')
        self.commandF = open("/sys/class/tacho-motor/motor"+self._portProps[1]+"/command", 'w')

        self.setCommand()

        self.bias = config.bias

        self._maxSpeed = micropython.const(config.maxSpeed)

        self.Ks, self.Kv, self.Ka = config.ff

        for key, value in kwargs.items():
            setattr(self, key, value)

    # @micropython.native
    def run(self, speed: float = 0, acc: float = 0) -> None:
        self.dutyCycle(copysign(self.Ks, speed) + self.Kv*speed + self.Ka*acc)

    # @micropython.native
    def stop(self) -> None:
        self.dutyCycle(0)

    # @micropython.native
    def getAngle(self) -> float:
        return self.count() / 2 + self.bias

    # @micropython.native
    def getRot(self) -> float:
        return self.getAngle() / 360

    # @micropython.native
    def getSpeed(self) -> float:
        return self.frequency() / 2

    # @micropython.native
    def reset(self, bias: float = 0) -> None:
        self.bias = self.count() / 2 + bias

    # @micropython.native
    def count(self) -> int:
        self.countF.seek(0)
        return int(self.countF.read()) * self.dir

    # @micropython.native
    def frequency(self) -> int:
        self.frequencyF.seek(0)
        return int(self.frequencyF.read()) * self.dir

    # @micropython.native
    def dutyCycle(self, dutyCycle: int) -> None:
        self.dutyCycleF.seek(0)
        if abs(dutyCycle) < 100:
            self.dutyCycleF.write(str(int(dutyCycle)))
        else:
            self.dutyCycleF.write(str(int(copysign(100, dutyCycle))))
        self.dutyCycleF.flush()

    # @micropython.native
    def setCommand(self, command: str = "run-direct"):
        self.commandF.seek(0)
        self.commandF.write(command)
        self.commandF.flush()

    # @micropython.native
    def analysis(self, save: bool = True, filename: str = 'analysis.log') -> tuple:

        times = []
        speeds = []
        dutys = []

        st = time()
        for duty in range(101):

            self.dutyCycle(duty)

            loopT = time()
            while time() - loopT < 0.25:

                times.append(time() - st)
                speeds.append(self.frequency() / 2)
                dutys.append(duty)

        self.dutyCycle(0)

        acc = []
        _speed = []
        _dutys = []

        dutys.append(101)

        for duty in range(101):

            index = dutys.index(duty)
            slice = speeds[index:dutys.index(duty+1)]

            maxV = max(slice)
            maxVIndex = slice.index(maxV) + index

            deltaT = times[maxVIndex] - times[index]

            if deltaT == 0:
                acc.append(0)
            else:
                acc.append(maxV / deltaT)

            _speed.append(maxV)
            _dutys.append(duty)

        if save:
            with open(filename, 'w') as f:
                f.write(str({'duty': _dutys, 'velocity': _speed, 'acceleration': acc}))

        return _dutys, _speed, acc


class Gyro:
    def __init__(self, config: object, **kwargs):

        self._port = micropython.const(config.port)
        self._portF = sensor_ports[self._port]

        self.dir = config.direction

        self.valueF = open("/sys/class/lego-sensor/sensor"+self._portF+"/value0", 'r')
        self.modeF = open("/sys/class/lego-sensor/sensor"+self._portF+"/mode", 'w')

        self.setMode()

        self.bias = self.read()*self.dir + config.bias

        for key, value in kwargs.items():
            setattr(self, key, value)

    # @micropython.native
    def getAngle(self) -> float:
        return self.read() * self.dir + self.bias

    # @micropython.native
    def getRate(self) -> float:
        self.setMode("GYRO-RATE")
        val = self.read() * self.dir
        self.setMode("GYRO-ANG")
        return val

    # @micropython.native
    def reset(self, angle: int = 0) -> None:
        self.bias = self.read() + angle

    # @micropython.native
    def calibrate(self) -> None:
        self.setMode("GYRO-CAL")
        sleep(2)
        self.setMode("GYRO-ANG")

    # @micropython.native
    def read(self) -> int:
        self.valueF.seek(0)
        return int(self.valueF.read())

    # @micropython.native
    def setMode(self, mode: str = "GYRO-ANG") -> None:
        self.modeF.seek(0)
        self.modeF.write(mode)
        self.modeF.flush()


class DualGyro(Gyro):
    def __init__(self, config1: dict, config2: dict, **kwargs):
        self.gyro1 = Gyro(config1)
        self.gyro2 = Gyro(config2)

        for key, value in kwargs.items():
            setattr(self, key, value)

    # @micropython.native
    def getAngle(self) -> float:
        return (self.gyro1.getAngle() + self.gyro2.getAngle()) / 2

    # @micropython.native
    def getRate(self) -> float:
        return (self.gyro1.getRate() + self.gyro2.getRate()) / 2

    # @micropython.native
    def reset(self, angle: int = 0) -> None:
        self.gyro1.reset(angle)
        self.gyro2.reset(angle)

    # @micropython.native
    def calibrate(self, angle: int = 0) -> None:
        self.gyro1.calibrate(angle)
        self.gyro2.calibrate(angle)


class LightSensor:
    def __init__(self, config: object, **kwargs):

        self._port = micropython.const(config.port)
        self._portF = sensor_ports[config.port]

        self.valueF = open("/sys/class/lego-sensor/sensor"+self._portF+"/value0", 'r')
        self.modeF = open("/sys/class/lego-sensor/sensor"+self._portF+"/mode", 'w')

        self.setMode()

        for key, value in kwargs.items():
            setattr(self, key, value)

    # @micropython.native
    def getReflect(self) -> int:
        return self.read()

    # @micropython.native
    def getColor(self) -> int:
        self.setMode('COL-COLOR')
        val = self.read()
        self.setMode("COL-REFLECT")
        return val

    # @micropython.native
    def read(self) -> int:
        self.valueF.seek(0)
        return int(self.valueF.read())

    # @micropython.native
    def setMode(self, mode: str = "COL-REFLECT") -> None:
        self.modeF.seek(0)
        self.modeF.write(mode)
        self.modeF.flush()


# @micropython.native
def MotorPorts() -> None:
    for dir in listdir("/sys/class/tacho-motor"):
        with open("/sys/class/tacho-motor/"+dir+"/address", 'r') as f:
            addr = f.read()
        motor_ports[addr[13]][1] = dir[5]


# @micropython.native
def SensorPorts() -> None:
    for dir in listdir("/sys/class/lego-sensor"):
        with open("/sys/class/lego-sensor/"+dir+"/address", 'r') as f:
            addr = f.read()
        sensor_ports[addr[12]] = dir[6]


def startup() -> None:
    MotorPorts()
    SensorPorts()


if __name__ != "__main__":
    startup()
