import micropython
from ev3devices_basic import LightSensor
from controllers import PIDController
from mytools import mean


class Sensorbase:
    def __init__(self, lLight: str, rLight: str, drivebase: object):

        self.ll = LightSensor(lLight)
        self.rl = LightSensor(rLight)

    # @micropython.nativex
    def box(self, Kpid: [float, float, float] = [0, 0, 0], rfl: int = 50) -> None:
        self.drivebase.gyro.setMode("GYRO-RATE")
        PID = PIDController(*Kpid, rfl)
        while True:
            PID.correction()

        self.drivebase.stop()

    # @micropython.native
    def LineFollow(self, speed: float, dist: float, rfl: int, side: int,
                   Kp: float, Ki: float, Kd: float, maxTime: float = None) -> None:

        PID = PIDController(Kp, Ki, Kd, rfl)
        cl = self.ll if side == -1 else self.rl

        start = mean(self.drivebase.getAngle())

        while (mean(self.drivebase.getAngle()) - start <
               dist / self.drivebase._wheelCircumference):

            corr = PID.correction(cl.getReflectedLight())
            self.drivebase.run_tank(speed + corr, speed - corr)

    # @micropython.native
    def LAGPID(self) -> None:
        # TODO: Line assisted gyro PID
        pass

# TODO
