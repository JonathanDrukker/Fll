import micropython
from gc import collect
from _thread import allocate_lock, start_new_thread
from time import time
from mytools import sleep
from controllers import RAMSETEController
from odometry import DiffrentialDriveOdometry
from drivebase import DriveBase
from ev3devices_advanced import Motor


class Runner:
    """
    Runner class - Used to run paths.
    Controls everything in the robot.
    Parameters:
        config: dict
    """

    lock = allocate_lock()

    def __init__(self, config: dict):

        self.drivebase = DriveBase(config)
        self.odometry = DiffrentialDriveOdometry(self.drivebase, self.lock)

        self.lm = Motor(config.motors.left)
        self.rm = Motor(config.motors.right)

    # @micropython.native
    def path(self, path: str, b: float, zeta: float, _log: bool = False) -> [list, int]:
        """
        Traverse a path.
        Handles events and markers.
        Path: multiple splines connected.
        Parameters:
            path: str - Path name
            b: float - Beta
            zeta: float - Zeta
            _log: bool - Log
        Returns:
            logs: list - Logs
            counter: int - Counter
        """
        counter = 0

        st = time()
        waypointsF = open("Paths/"+path+".cvs", "r")
        print("File open time:", time()-st)

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
        """
        Traverse a spline.
        Parameters:
            waypointsFile: object - Waypoints file
            RAMSETE: RAMSETEController - RAMSETE controller
            _log: bool - Log
        Returns:
            log: list - Log
            count: int - Counter
            waypoint: dict - Waypoint
        """
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

            self.drivebase.run_tank(Vl, Vr, waypoint[6], waypoint[7])

            if _log:
                log.append((cTime, currentX, currentY, currentTheata, self.odometry.getPhi()))

        self.drivebase.run_tank(0, 0)
        return log, count

    # @micropython.native
    async def stopEventsHandler(self, stopEvent: dict) -> None:
        """
        Handle stop events.
        Parameters:
            stopEvent: dict - Stop event
        """
        if stopEvent["waitBehavior"] == "None":
            self.commands(stopEvent["commands"], stopEvent["execBehavior"])
        elif stopEvent["waitBehavior"] == "Before":
            sleep(stopEvent["waitTime"])
            self.commands(stopEvent["commands"], stopEvent["execBehavior"])
        elif stopEvent["waitBehavior"] == "After":
            self.commands(stopEvent["commands"], stopEvent["execBehavior"])
            sleep(stopEvent["waitTime"])
        elif stopEvent["waitBehavior"] == "Minimum":
            await self.commands(stopEvent["commands"], stopEvent["execBehavior"])
            await sleep(stopEvent["waitTime"])

    # @micropython.native
    def markersHandler(self, markers: tuple, cTime: float) -> None:
        """
        Handle markers.
        Parameters:
            markers: tuple - Markers
            cTime: float - Current time
        """
        sleep(markers[0] - cTime())
        self.commands(markers[1], "Parallel")
        self.markersHandler(markers[2:], cTime+markers[0])
        # TODO: make not dumb solution

    # @micropython.native
    def commands(self, commands: list, execBehavior: str) -> None:
        """
        Execute commands.
        Parameters:
            commands: list - Commands
            execBehavior: str - Execution behavior
        """
        if execBehavior == "Parallel":
            for command in commands:
                start_new_thread(exec, (command))
        else:
            for command in commands:
                exec(command)

    # @micropython.native
    def getTargetWaypoint(self, cTime: float, file: object) -> dict:
        """
        Get the target waypoint.
        Parameters:
            cTime: float - Current time
            file: object - File
        Returns:
            waypoint: dict - Waypoint
        """
        if cTime - self.lastWaypoint['time'] > 0:
            return self.lastWaypoint
        while True:
            data = file.readline()
            if data == '':
                return None
            if int(data[:data.index(',')]) - cTime > 0:
                self.lastWaypoint = eval(data)
                return self.lastWaypoint
