from time import time


class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, target: float, feedback: float = 0):

        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.target = target

        self.startTime = time()
        self.lastTime = 0

        self.lastError = target-feedback

        self.integral = 0

    def correction(self, feedback: float, error=None) -> float:

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
