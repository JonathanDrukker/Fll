from os import listdir
from time import sleep
from typing import NoReturn

motor_ports = {'A': ['0', None], 'B': ['1', None], 'C': ['2', None], 'D': ['3', None]}
sensor_ports = {'1': None, '2': None, '3': None, '4': None}


class Motor:
    def __init__(self, Port: str, Direction: int, bias: float = 0, maxSpeed: float = 1000):

        self._port = Port
        self._portProps = motor_ports[Port]
        self.dir = Direction

        self.countF = open("/sys/bus/iio/devices/iio:device1/in_count"+self._portProps[0]+"_raw", 'r')
        self.frequencyF = open("/sys/bus/iio/devices/iio:device1/in_frequency"+self._portProps[0]+"_input", 'r')
        self.dutyCycleF = open("/sys/class/tacho-motor/motor"+self._portProps[1]+"/duty_cycle_sp", 'w')
        self.commandF = open("/sys/class/tacho-motor/motor"+self._portProps[1]+"/command", 'w')

        self.commandF.write("run-direct")

        self.bias = bias

        self.maxSpeed = maxSpeed

    def run(self, speed: float = 0) -> NoReturn:
        self.dutyCycle(speed / self.maxSpeed * 100)

    def stop(self) -> NoReturn:
        self.dutyCycle(0)

    def getAngle(self) -> float:
        return self.count() / 2 + self.bias

    def getRot(self) -> float:
        return self.getAngle() / 360

    def getSpeed(self) -> float:
        return self.frequency() / 2

    def reset(self, angle: float = 0) -> NoReturn:
        self.bias = self.count() / 2 + angle

    def count(self) -> int:
        self.countF.seek(0)
        return int(self.countF.read()) * self.dir

    def frequency(self) -> int:
        self.frequencyF.seek(0)
        return int(self.frequencyF.read()) * self.dir

    def dutyCycle(self, dutyCycle: int) -> NoReturn:
        self.dutyCycleF.seek(0)
        self.dutyCycleF.write(str(dutyCycle))


class Gyro:
    def __init__(self, Port: str, Direction: int, bias: float = 0):

        self._port = Port
        self._portF = sensor_ports[Port]

        self.dir = Direction

        self.valueF = open("/sys/class/lego-sensor/sensor"+self._portF+"/value0", 'r')
        self.modeF = open("/sys/class/lego-sensor/sensor"+self._portF+"/mode", 'w')

        self.modeF.write("GYRO-ANG")

        self.bias = self.getAngle() + bias

    def getAngle(self) -> float:
        return self.read() * self.dir + self.bias

    def getRate(self) -> float:
        self.setMode("GYRO-RATE")
        val = self.read() * self.dir
        self.setMode("GYRO-ANG")
        return val

    def reset(self, angle: int = 0) -> NoReturn:
        self.bias = self.read() + angle

    def calibrate(self, angle: int = 0) -> NoReturn:
        self.setMode("GYRO-CAL")
        self.setMode("GYRO-ANG")
        sleep(0.1)
        self.reset(angle)

    def read(self) -> int:
        self.valueF.seek(0)
        return int(self.valueF.read())

    def setMode(self, mode: str = "GYRO-ANG") -> NoReturn:
        self.modeF.seek(0)
        self.modeF.write(mode)


class DualGyro(Gyro):
    def __init__(self, Port1: str, Direction1: int, Port2: str, Direction2: int, bias: float = 0):
        self.gyro1 = Gyro(Port1, Direction1, bias)
        self.gyro2 = Gyro(Port2, Direction2, bias)

    def getAngle(self) -> float:
        return (self.gyro1.getAngle() + self.gyro2.getAngle()) / 2

    def getRate(self) -> float:
        return (self.gyro1.getRate() + self.gyro2.getRate()) / 2

    def reset(self, angle: int = 0) -> NoReturn:
        self.gyro1.reset(angle)
        self.gyro2.reset(angle)

    def calibrate(self, angle: int = 0) -> NoReturn:
        self.gyro1.calibrate(angle)
        self.gyro2.calibrate(angle)


class LightSensor:
    def __init__(self, Port: str):

        self._port = Port
        self._portF = sensor_ports[Port]

        self.valueF = open("/sys/class/lego-sensor/sensor"+self._portF+"/value0", 'r')
        self.modeF = open("/sys/class/lego-sensor/sensor"+self._portF+"/mode", 'w')

        self.modeF.write("COL-REFLECT")

    def getReflect(self) -> int:
        return self.read()

    def getColor(self) -> int:
        self.setMode('COL-COLOR')
        val = self.read()
        self.setMode("COL-REFLECT")
        return val

    def read(self) -> int:
        self.valueF.seek(0)
        return int(self.valueF.read())

    def setMode(self, mode: str = "COL-REFLECT") -> NoReturn:
        self.modeF.seek(0)
        self.modeF.write(mode)


def MotorPorts() -> NoReturn:
    for dir in listdir("/sys/class/tacho-motor"):
        with open("/sys/class/tacho-motor/"+dir+"/address", 'r') as f:
            addr = f.read()
        motor_ports[addr[13]][1] = dir[5]


def SensorPorts() -> NoReturn:
    for dir in listdir("/sys/class/lego-sensor"):
        with open("/sys/class/lego-sensor/"+dir+"/address", 'r') as f:
            addr = f.read()
        sensor_ports[addr[12]] = dir[6]


def startup() -> NoReturn:
    MotorPorts()
    SensorPorts()


if __name__ != "__main__":
    startup()
