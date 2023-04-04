import _thread
from ev3devices import Motor, DualGyro
from math import sin, cos, pi
from time import time
from mytools import mean, thread # noqa
from Comms.message import Message


class DriveBase:

    lock = _thread.allocate_lock()

    def __init__(self, lMotor: tuple, rMotor: tuple, gyro: tuple,
                 wheelRad: float, DBM: float, position=[0, 0, 0]):

        self.lm = Motor(*lMotor)
        self.rm = Motor(*rMotor)
        self.gyro = DualGyro(*gyro)

        self.wheelRad = wheelRad
        self.wheelCircumference = 2*pi*wheelRad

        self.DBM = DBM
        self.halfDBM = DBM/2

        self.position = position
        self.updatePosition()

    def trackPath(self, path, Kp, spareTime=0, _print=True, _client=None):

        self.resetPosition([path[0]['x'], path[0]['y'], path[0]['theata']])
        self.lm.resetPID(); self.rm.resetPID()

        for waypoint in self.getTargetPos(path, spareTime):

            x, y = waypoint['x'], waypoint['y']

            with self.lock:
                currentX, currentY, currentTheata = self.position

            Vx = (x - currentX) * Kp
            Vy = (y - currentY) * Kp

            Vl, Vr = self.calc_velocity(Vx, Vy, currentTheata)

            self.run_tank(self.motorSpeed(Vl + waypoint['Vl']),
                          self.motorSpeed(Vr + waypoint['Vr']))

            if _print:
                print("Pos: ", currentX, currentY, currentTheata,
                      "\nTarget: ", x, y,
                      "\nVx:", Vx, "Vy:", Vy, "\nVl: ", Vl, "Vr: ", Vr)

            if _client:
                with self.lock:
                    _client.send(Message('graphData0', str((currentX, currentY))))
                    _client.send(Message('graphData1', str((x, y))))

    def calc_velocity(self, Vx, Vy, theata):

        # TODO: Add a way to calculate the velocity of the robot
        Vl, Vr = 0, 0

        return Vl, Vr

    def motorSpeed(self, V):
        return V/self.wheelCircumference*360

    def run_tank(self, Vl, Vr):
        self.lm.PIDRun(Vl)
        self.rm.PIDRun(Vr)

    @thread
    def updatePosition(self):

        self.resetPos = None

        past_lm_pos, past_rm_pos = self.lm.getRot(), self.rm.getRot()
        past_theata = self.gyro.getRawRadians()

        while True:

            lm_pos, rm_pos = self.lm.getRot(), self.rm.getRot()
            theata = self.gyro.getRawRadians()

            Vl = lm_pos - past_lm_pos
            Vr = rm_pos - past_rm_pos

            V = self.wheelCircumference/2*(Vr + Vl)
            # omega = mean(self.wheelCircumference / self.DBM * (Vr - Vl), theata-past_theata)
            omega = self.wheelCircumference / self.DBM * (Vr - Vl)

            theata = self.position[2] + omega
            if theata > pi: theata -= 2*pi
            elif theata < -pi: theata += 2*pi

            x, y = V*cos(theata) + self.position[0], V*sin(theata) + self.position[1]

            with self.lock: self.position = [x, y, theata]

            past_lm_pos, past_rm_pos = lm_pos, rm_pos
            past_theata = theata # noqa

            if self.__resetPos:
                with self.lock: self.position = self.__resetPos
                self.__resetPos = None

    def resetPosition(self, position=[0, 0, 0]):
        self.__resetPos = position

    @staticmethod
    def getTargetPos(path, spareTime=0):

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

        cTime = time()-startTime
        timeError = path[-1]['time']+spareTime - cTime

        while timeError > 0:
            cTime = time()-startTime
            timeError = path[-1]['time']+spareTime - cTime
            yield path[-1]
