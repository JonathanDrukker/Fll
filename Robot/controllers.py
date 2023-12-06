import micropython
from time import time
from math import pi, sin, cos, sqrt


class PIDController:
    """
    PID Controller
    Parameters:
        Kp: float - Proportional gain
        Ki: float - Integral gain
        Kd: float - Derivative gain
        target: float - Target value
        feedback: float - Feedback value
    """
    def __init__(self, Kp: float, Ki: float, Kd: float, target: float, feedback: float = 0):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = target

        self.startTime = time()
        self.lastTime = 0.0

        self.lastError = 0

        self.integral = 0.0

    @micropython.native
    def correction(self, feedback: float, error: float = None) -> float:
        """
        Calculate the correction value
        Parameters:
            feedback: float - Feedback value
            error: float - Error value
        Returns:
            correction: float - Correction value
        """

        if error is None:
            error = self.target-feedback

        _time = time()
        dt = _time-self.lastTime
        if (dt == 0): dt += 0.0001

        correction = self.calc_correction(error, dt)

        self.lastTime = _time
        self.lastError = error

        return correction

    @micropython.native
    def calc_correction(self, error: float, dt: float) -> float:
        """
        Calculate the correction value
        Parameters:
            error: float - Error value
            dt: float - Time delta
        Returns:
            correction: float - Correction value
        """

        p = error
        self.integral += (error+self.lastError)/2*dt
        d = (error-self.lastError)/dt

        return self.Kp*p + self.Ki*self.integral + self.Kd*d

    @micropython.native
    def setGains(self, Kp: float, Ki: float, Kd: float) -> None:
        """
        Set the gains
        Parameters:
            Kp: float - Proportional gain
            Ki: float - Integral gain
            Kd: float - Derivative gain
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd


class RAMSETEController:
    """
    RAMSETE Controller
    Parameters:
        b: float - Beta
        zeta: float - Zeta
        halfDBM: float - Half of the DBM
    """
    def __init__(self, b: float, zeta: float, halfDBM: float):

        self.b = b
        self.zeta = zeta

        self.halfDBM = micropython.const(halfDBM)

    @micropython.native
    def correction(self, Vx: float, Vy: float, theata: float, V: float, Omega: float, theata_d: float) -> [float, float]:
        """
        Calculate the correction value
        Parameters:
            Vx: float - X velocity
            Vy: float - Y velocity
            theata: float - Theata
            V: float - Velocity
            Omega: float - Omega
            theata_d: float - Theata delta
        Returns:
            [leftCorr, rightCorr]: [float, float] - Correction value
        """

        cosTheata, sinTheata = cos(theata), sin(theata)

        Ex = cosTheata*Vx + sinTheata*Vy
        Ey = cosTheata*Vy - sinTheata*Vx

        Etheata = theata_d - theata
        if (Etheata > pi): Etheata -= 2*pi
        elif (Etheata < -pi): Etheata += 2*pi

        k = 2*self.zeta*sqrt(Omega**2 + self.b*V**2)

        v = V*cos(Etheata) + k*Ex
        if (Etheata != 0):
            Omega += k*Etheata + (self.b*V*sin(Etheata)*Ey)/Etheata
        Omega *= self.halfDBM

        return v + Omega, v - Omega

    @micropython.native
    def setGains(self, b: float, zeta: float) -> None:
        """
        Set the gains
        Parameters:
            b: float - Beta
            zeta: float - Zeta
        """
        self.b = b
        self.zeta = zeta
