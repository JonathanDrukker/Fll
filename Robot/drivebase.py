import _thread
from ev3devices import Motor, DualGyro
from math import sin, cos, atan2, pi, sqrt
from time import time
from mytools import mean, thread # noqa
from Comms.message import Message


class DriveBase:

    lock = _thread.allocate_lock()

    def __init__(self, lMotor: tuple, rMotor: tuple, gyro: tuple,
                 wheelRad: float, DBM: float,
                 x: float, y: float, theata: float):

        self.lm = Motor(*lMotor)
        self.rm = Motor(*rMotor)
        self.gyro = DualGyro(*gyro)

        self.wheelRad = wheelRad
        self.wheelCircumference = 2*pi*wheelRad

        self.DBM = DBM
        self.halfDBM = DBM/2

        self.x, self.y = x, y
        self.theata = theata
        self.updatePosition()

    def trackPath(self, path, b, zeta, _print=True, _client=None):

        self.resetPos(path[0]['x'], path[0]['y'], path[0]['theata'])

        for waypoint in self.getTargetPos(path):

            with self.lock:
                currentX, currentY = self.x, self.y
                currentTheata = self.theata

            Vx, Vy = waypoint['x'] - currentX, waypoint['y'] - currentY

            Vl, Vr = self.calc_velocity(Vx, Vy, currentTheata, self.halfDBM,
                                        waypoint['V'], waypoint['omega'], b, zeta)

            self.run_tank(self.motorSpeed(Vl), self.motorSpeed(Vr))

            if _print:
                print("Pos: ", currentX, currentY, currentTheata,
                      "\nTarget: ", waypoint['x'], waypoint['y'],
                      "\nVx:", Vx, "Vy:", Vy, "\nVl: ", Vl, "Vr: ", Vr)

            if _client:
                with self.lock:
                    _client.send(Message('graphData0', str((currentX, currentY))))
                    _client.send(Message('graphData1', str((waypoint['x'], waypoint['y']))))

    @staticmethod
    def calc_velocity(Vx, Vy, theata, halfDBM, V, Omega, b, zeta):

        def sign(num):
            if num > 0: return 1
            elif num < 0: return -1
            else: return 0

        k1 = 2*zeta*sqrt(Omega**2 + b*V**2)
        k2 = b*abs(V)

        Vtheta = atan2(Vy, Vx) - theata
        if (Vtheta > pi): Vtheta -= 2*pi
        elif (Vtheta < -pi): Vtheta += 2*pi

        v = V*cos(Vtheta) + k1*(cos(theata)*Vx + sin(theata)*Vy)
        omega = (Omega + k2*sign(V) * (cos(theata)*Vy - sin(theata)*Vx) + k1*Vtheta) * halfDBM

        return v - omega, v + omega

    def motorSpeed(self, V):
        return V/self.wheelCircumference*360

    def run_tank(self, Vl, Vr):
        self.lm.PIDRun(Vl)
        self.rm.PIDRun(Vr)

    @thread
    def updatePosition(self):
        self.__resetPos = None

        past_lm_pos, past_rm_pos = self.lm.getRot(), self.rm.getRot()
        past_theata = self.gyro.getRawRadians()

        while True:

            lm_pos, rm_pos = self.lm.getRot(), self.rm.getRot()
            theata = self.gyro.getRawRadians()

            Vl = lm_pos - past_lm_pos
            Vr = rm_pos - past_rm_pos

            V = self.wheelCircumference/2*(Vr + Vl)
            omega = self.wheelCircumference / self.DBM * (Vr - Vl)
            # omega = mean(omega, theata-past_theata)

            theata = self.theata + omega
            if (theata > pi): theata -= 2*pi
            elif (theata < -pi): theata += 2*pi

            x = self.x + V*cos(theata)
            y = self.y + V*sin(theata)

            with self.lock:
                self.x, self.y, self.theata = x, y, theata

            if self.__resetPos:
                with self.lock:
                    self.x, self.y, self.theata = self.__resetPos
                self.__resetPos = None

            past_lm_pos, past_rm_pos = lm_pos, rm_pos
            past_theata = theata # noqa

    def resetPos(self, x, y, theata):
        self.__resetPos = [x, y, theata]

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
