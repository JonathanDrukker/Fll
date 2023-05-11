from time import time
from math import sin, cos, sqrt, pi


class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, target: float, feedback: float = 0):

        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.target = target

        self.startTime = time()
        self.lastTime = 0

        self.lastError = target-feedback

        self.integral = 0

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

    def calc_correction(self, error: float, dt: float) -> float:

        p = error
        self.integral += (error+self.lastError)/2*dt
        d = (error-self.lastError)/dt

        return self.Kp*p + self.Ki*self.integral + self.Kd*d


class RAMSETEController:
    def __init__(self, b, zeta, halfDBM):

        self.b = b
        self.zeta = zeta

        self.halfDBM = halfDBM

    def correction(self, Vx, Vy, theata, V, Omega, theata_d):

        cosTheata = cos(theata); sinTheata = sin(theata)

        Ex = cosTheata*Vx + sinTheata*Vy
        Ey = -sinTheata*Vx + cosTheata*Vy

        Etheata = theata_d - theata
        if (Etheata > pi): Etheata -= 2*pi
        elif (Etheata < -pi): Etheata += 2*pi
        if (Etheata == 0): Etheata = 0.00001

        k = 2*self.zeta*sqrt(Omega**2 + self.b*V**2)

        v = V*cos(Etheata) + k*Ex
        omega = (Omega + k*Etheata + (self.b*V*sin(Etheata)*Ey)/Etheata) * self.halfDBM

        return v - omega, v + omega
