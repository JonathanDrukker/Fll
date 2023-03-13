import _thread
from parameters import Position
from ev3devices import Motor, DualGyro
from math import tan, atan2, asin, degrees, radians, pi, sqrt
from time import time
from mymath import mean


class DriveBase:

    runUpdatePos = True
    lock = _thread.allocate_lock()

    def __init__(self, lMotor: tuple, rMotor: tuple, gyro: tuple,
                 wheelRad: float, DBM: float, position=Position(0, 0, 0)):

        self.lm = Motor(*lMotor)
        self.rm = Motor(*rMotor)
        self.gyro = DualGyro(*gyro)

        self.wheelRad = wheelRad
        self.wheelCircumference = 2*pi*wheelRad
        self.DBM = DBM

        self.position = position

        _thread.start_new_thread(self.updatePos, ())

    def trackPath(self, path, spareTime=0, _print=True):

        for waypoint, timeError in self.getTargetPos(path, spareTime):

            targetPos = Position(*waypoint['pose'])

            with self.lock:
                currentPos = self.position

            V, omega = self.calc_velocity(currentPos, targetPos, timeError)
            self.run_velocity(V, omega)

            if _print:
                print("Pos: {", currentPos.__str__(), "}")
                print("Target: {", targetPos.__str__(), "}")
                print("V: ", V, ", W: ", omega)

    def calc_velocity(self, currentPos: Position, targetPos: Position, dt: float):
        Vx = targetPos.x - currentPos.x
        Vy = targetPos.y - currentPos.y

        dist = sqrt(Vx**2 + Vy**2)

        alpha = degrees(atan2(Vy, Vx)) - currentPos.theata

        h = dist/2*tan(radians(alpha))
        r = (dist/2) + (h**2/(2*dist))
        c = 2*pi*r

        print(r)

        beta = degrees(2 * asin(dist/(2*r)))

        V = (c*(beta/360))/dt
        omega = beta/dt

        return V, omega

    def resetPos(self, position: Position):
        self.runUpdatePos = False
        with self.lock:
            self.position = position
        self.runUpdatePos = True
        _thread.start_new_thread(self.getPosition, ())

    def updatePos(self):

        past_lm_pos = self.lm.getRawAngle() / 360
        past_rm_pos = self.rm.getRawAngle() / 360

        past_angle = self.gyro.getRawAngle()

        while self.runUpdatePos:

            lm_pos = self.lm.getRawAngle() / 360
            rm_pos = self.rm.getRawAngle() / 360

            angle = self.gyro.getRawAngle()

            delta_lm_pos = lm_pos - past_lm_pos
            delta_rm_pos = rm_pos - past_rm_pos

            V = self.wheelCircumference/2*(delta_rm_pos + delta_lm_pos)
            omega = mean(degrees(self.wheelCircumference / self.DBM *
                         (delta_rm_pos - delta_lm_pos)),
                         angle-past_angle)

            with self.lock:
                self.position.__change__(V, omega)

            past_lm_pos = lm_pos
            past_rm_pos = rm_pos

            past_angle = angle

    def getTargetPos(self, path, spareTime=0):

        startTime = time()
        cTime = 0
        index = 0

        while cTime <= path[index]['time']:
            cTime = time()-startTime

            for index, dict in enumerate(path[index:]):
                timeError = dict['time'] - cTime
                if timeError > 0:
                    yield path[index], timeError

        while cTime <= path[-1]['time']+spareTime:
            cTime = time()-startTime
            timeError = path[-1]['time']+spareTime - cTime
            yield path[-1], timeError

    def run_velocity(self, V, omega):

        V = V/self.wheelCircumference*360
        omega /= 2

        self.lm.run(V-omega)
        self.rm.run(V+omega)

    def run_tank(self, left, right):
        self.lm.run(left)
        self.rm.run(right)

    def stop(self):
        self.lm.trueStop()
        self.rm.trueStop()
