import _thread
from ev3devices import Motor, DualGyro
from math import sin, cos, pi
from time import time
from mytools import mean, thread # noqa
from Comms.message import Message
from controllers import RAMSETEController


class DriveBase:

    lock = _thread.allocate_lock()

    def __init__(self, lMotor: tuple, rMotor: tuple, gyro: tuple,
                 wheelDiameter: float, DBM: float,
                 x: float, y: float, theata: float):

        self.lm = Motor(*lMotor)
        self.rm = Motor(*rMotor)
        self.gyro = DualGyro(*gyro)

        self.wheelRad = wheelDiameter/2
        self.wheelDiameter = wheelDiameter
        self.wheelCircumference = pi*wheelDiameter

        self.DBM = DBM
        self.halfDBM = DBM/2

        self.x, self.y = x, y
        self.theata = theata
        self.odometry()

    def trackPath(self, path, b, zeta, _print=True, _logfile=None, _client=None):

        self.resetPos(path[0]['x'], path[0]['y'], path[0]['theata'])
        RAMSETE = RAMSETEController(b, zeta, self.halfDBM)

        for waypoint in self.getTargetPos(path):

            with self.lock:
                currentX, currentY = self.x, self.y
                currentTheata = self.theata

            Vx, Vy = waypoint['x'] - currentX, waypoint['y'] - currentY

            Vl, Vr = RAMSETE.correction(Vx, Vy, currentTheata,
                                        waypoint['V'], waypoint['omega'], waypoint['theata'])

            self.run_tank(self.motorSpeed(Vl), self.motorSpeed(Vr))

            data = str({'time': time(),
                        'robot': {
                        'Pose': (currentX, currentY, currentTheata),
                        'gyroRate': self.gyro.getSpeed(),
                        'V': (self.lm.getSpeed(), self.rm.getSpeed()),
                        'Vtarget': (self.lm.PID.target, self.rm.PID.target)},
                        'waypoint': waypoint})

            if _print: print(data)
            if _logfile: _logfile.write(data + '\n')
            if _client:
                with self.lock: _client.send(Message('log', data))

        self.run_tank(0, 0)

    def motorSpeed(self, V):
        return V/self.wheelCircumference*360

    def run_tank(self, Vl, Vr):
        self.targetVl, self.targetVr = Vl, Vr
        self.lm.PIDRun(Vl)
        self.rm.PIDRun(Vr)

    @thread
    def odometry(self):
        self.__resetPos = None

        past_lm_pos, past_rm_pos = self.lm.getRot(), self.rm.getRot()

        while True:

            lm_pos, rm_pos = self.lm.getRot(), self.rm.getRot()

            Vl = lm_pos - past_lm_pos
            Vr = rm_pos - past_rm_pos

            self.Velocity = self.wheelCircumference/2*(Vr + Vl)
            self.omega = self.wheelCircumference / self.DBM * (Vr - Vl)

            # theata = mean(self.theata + omega, self.gyro.getRawRadians())
            theata = self.theata + self.omega
            if (theata > pi): theata -= 2*pi
            elif (theata < -pi): theata += 2*pi

            x = self.x + self.Velocity*cos(theata)
            y = self.y + self.Velocity*sin(theata)

            with self.lock:
                self.x, self.y, self.theata = x, y, theata

            if self.__resetPos:
                with self.lock:
                    self.x, self.y, self.theata = self.__resetPos
                self.__resetPos = None

            past_lm_pos, past_rm_pos = lm_pos, rm_pos

    def resetPos(self, x, y, theata):
        self.__resetPos = [x, y, theata]
        self.gyro.resetAngle(theata)

    @staticmethod
    def getTargetPos(path):

        startTime = time()
        cTime = 0
        index = 0

        while cTime <= path[-1]['time']:
            cTime = time()-startTime

            for index, dict in enumerate(path[index:]):
                timeError = dict['time'] - cTime

                if timeError > 0:
                    yield dict
                    break
