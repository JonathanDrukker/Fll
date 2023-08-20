import micropython
from gc import collect
from _thread import allocate_lock
from threading import List, NoReturn
from time import time
from controllers import RAMSETEController
from odometry import DiffrentialDriveOdometry
from drivebase import DriveBase


class Runner:

    lock = allocate_lock()

    def __init__(self, lMotor: tuple, rMotor: tuple, gyro: tuple,
                 wheelRadius: float, DBM: float) -> NoReturn:

        self.drivebase = DriveBase(lMotor, rMotor, gyro, wheelRadius, DBM, self.lock)
        self.odometry = DiffrentialDriveOdometry(self.drivebase, self.lock)

    @micropython.native
    def path(self, path: tuple, b: float, zeta: float, Kacc: float, _log: bool = False) -> List[list, int]:
        counter = 0

        logs = []

        RAMSETE = RAMSETEController(b, zeta, self._halfDBM)

        for spline in path:
            log, count = self.spline(spline, RAMSETE, Kacc, _log)

            counter += count
            logs.append(log)

        return logs, counter

    @micropython.native
    def spline(self, path: tuple, RAMSETE: RAMSETEController, Kacc: float, _log: bool = False) -> List[list, int]:
        count = 0
        collect()

        if _log:
            log = []
        else:
            log = None

        startTime = time() - path[0]['time']
        cTime = 0.0
        index = 0

        while cTime <= path[-1]['time']:
            cTime = time() - startTime
            index, waypoint = self.getTargetWaypoint(cTime, index, path)

            currentX, currentY = self.odometry.getPos2d()
            currentTheata = self.gyro.getRadians()

            Vx, Vy = waypoint['x'] - currentX, waypoint['y'] - currentY

            Vl, Vr = RAMSETE.correction(Vx, Vy, currentTheata,
                                        waypoint['V'], waypoint['omega'], waypoint['theata'])

            self.run_tank(self.motorSpeed(
                Vl + Kacc*waypoint['accL']), self.motorSpeed(Vr + Kacc*waypoint['accR']))

            if _log:
                log.append({'time': cTime,
                            'robot': {
                                'Pose': (currentX, currentY, currentTheata),
                                'V': (self.lm.getSpeed(), self.rm.getSpeed())},
                            'waypoint': waypoint})

        self.run_tank(0, 0)
        return log, count

    @micropython.native
    @staticmethod
    def getTargetWaypoint(cTime: float, sIndex: int, path: tuple) -> List[dict, int]:
        for index, waypoint in enumerate(path[sIndex:]):
            if (waypoint['time'] - cTime > 0):
                return index+sIndex, waypoint
        else:
            return index+sIndex, path[-1]
