import micropython
from time import time
from math import pi, sin, cos, sqrt


class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, target: float, feedback: float = 0):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = target

        self.startTime = time()
        self.lastTime = 0.0

        self.lastError = target-feedback

        self.integral = 0.0

    # @micropython.native
    def correction(self, feedback: float, error: float = None) -> float:

        if (error is None):
            error = self.target-feedback

        _time = time()
        dt = _time-self.lastTime
        if (dt == 0): dt += 0.0001

        correction = self.calc_correction(error, dt)

        self.lastTime = _time
        self.lastError = error

        return correction

    # @micropython.native
    def calc_correction(self, error: float, dt: float) -> float:

        p = error
        self.integral += (error+self.lastError)/2*dt
        d = (error-self.lastError)/dt

        return self.Kp*p + self.Ki*self.integral + self.Kd*d

    # @micropython.native
    def setGains(self, Kp: float, Ki: float, Kd: float) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd


class RAMSETEController:
    def __init__(self, b: float, zeta: float, halfDBM: float):

        self.b = b
        self.zeta = zeta

        self.halfDBM = micropython.const(halfDBM)

    # @micropython.native
    def correction(self, Vx: float, Vy: float, theata: float, V: float, Omega: float, theata_d: float) -> [float, float]:

        cosTheata, sinTheata = cos(theata), sin(theata)

        Ex = cosTheata*Vx + sinTheata*Vy
        Ey = cosTheata*Vy - sinTheata*Vx

        Etheata = theata_d - theata
        if (Etheata > pi): Etheata -= 2*pi
        elif (Etheata < -pi): Etheata += 2*pi
        elif (Etheata == 0): Etheata = 0.00001

        k = 2*self.zeta*sqrt(Omega**2 + self.b*V**2)

        v = V*cos(Etheata) + k*Ex
        omega = (Omega + k*Etheata + (self.b*V*sin(Etheata)*Ey)/Etheata) * self.halfDBM

        return v - omega, v + omega

    # @micropython.native
    def setGains(self, b: float, zeta: float) -> None:
        self.b = b
        self.zeta = zeta
