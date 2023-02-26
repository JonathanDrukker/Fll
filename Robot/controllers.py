from pybricks.tools import StopWatch


class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, target: float):

        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.target = target

        self.timer = StopWatch()
        self.lastTime = 0

        self.lastError = 0

        self.integral = 0

    def correction(self, feedback: float, error=None) -> float:

        time = self.timer.time()/1000

        if (error is None):
            error = self.target-feedback
        dt = time-self.lastTime

        if (dt == 0):
            dt += 0.0001

        correction = self.calc_correction(error, dt)

        self.lastTime = time
        self.lastError = error

        return correction

    def calc_correction(self, error: float, dt: float) -> float:

        p = error
        self.integral += (error+self.lastError)/2*dt
        d = (error-self.lastError)/dt

        return self.Kp*p + self.Ki*self.integral + self.Kd*d
