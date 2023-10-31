import micropython
from gc import collect
from _thread import allocate_lock
from time import time
from controllers import RAMSETEController
from odometry import DiffrentialDriveOdometry
from drivebase import DriveBase


class Runner:

    lock = allocate_lock()

    def __init__(self, config: dict):

        self.drivebase = DriveBase(config, self.lock)
        self.odometry = DiffrentialDriveOdometry(self.drivebase, self.lock)

    # @micropython.native
    def path(self, path: str, b: float, zeta: float, _log: bool = False) -> [list, int]:
        counter = 0

        waypointsF = open("Paths/"+path+".cvs", "r")
        self.waypointsF.readline()
        self.lastWaypoint = self.waypointsF.readline()

        with open("Paths/"+path+".events", "r") as f:
            stopEvents, markers = eval(f.readline())
        self.markersHandler(markers)

        logs = []

        RAMSETE = RAMSETEController(b, zeta, self.drivebase._halfDBM)
        self.odometry.resetPos(self.lastWaypoint[1], self.lastWaypoint[2], self.lastWaypoint[3])

        while True:
            log, count = self.spline(waypointsF, RAMSETE, _log)

            counter += count
            logs += log

            self.stopEventsHandler(len(logs), stopEvents)

        return logs, counter

    # @micropython.native
    def spline(self, waypointsFile: object, RAMSETE: RAMSETEController, _log: bool = False) -> [list, int, dict]:
        collect()
        count = 0

        self.lastWaypoint = self.getTargetWaypoint(0, waypointsFile)

        if _log:
            log = []
        else:
            log = None

        startTime = time() - self.lastWaypoint[0]
        cTime = 0.0

        while True:

            cTime = time() - startTime
            waypoint = self.getTargetWaypointFromFile(cTime, waypointsFile)
            if waypoint is None:
                break

            currentX, currentY, currentTheata = self.odometry.getPos2d()
            Vx, Vy = waypoint[1] - currentX, waypoint[2] - currentY

            Vl, Vr = RAMSETE.correction(Vx, Vy, currentTheata,
                                        waypoint[4], waypoint[5], waypoint[3])

            self.drivebase.run_tank(Vl, waypoint[6], Vr, waypoint[7])

            if _log:
                log.append({'time': cTime,
                            'robot': {
                                'Pose': [currentX, currentY, currentTheata],
                                'Phi': self.odometry.getRot()},
                            'waypoint': waypoint})

        self.drivebase.run_tank(0, 0)
        return log, count

    # @micropython.native
    def stopEventsHandler(self, index: int, stopEvents: tuple) -> None:
        pass

    # @micropython.native
    def markersHandler(self, markers: tuple) -> None:
        pass

    # @micropython.native
    def getTargetWaypoint(self, cTime: float, file: object) -> dict:
        if cTime - self.lastWaypoint['time'] > 0:
            return self.lastWaypoint
        while True:
            data = file.readline()
            if data == '':
                return None
            if int(data[:data.index(',')]) - cTime > 0:
                return eval(data)
