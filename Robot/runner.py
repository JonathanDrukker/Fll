import micropython
from gc import collect
from _thread import allocate_lock, start_new_thread
from mytools import sleep, Timer
from time import time
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
    def path(self, filename: str, b: float, zeta: float, _log: bool = False) -> [list, int]:
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

        print("Loading Path..."); st = time()
        with open("Paths/"+filename+".path", "r") as f:
            path = eval(f.read())
        print("Loaded path in", time()-st)

        with open("Paths/"+filename+".events", "r") as f:
            stopEvents, markers = eval(f.readline())

        logs = []

        RAMSETE = RAMSETEController(b, zeta, self.drivebase._halfDBM)

        self.odometry.resetPos(path[0][0][1], path[0][0][2], path[0][0][3])
        self.odometry.start()

        self.timer = Timer()

        for spline in path:

            self.markersHandler(markers[len(logs)])

            log, count = self.spline(spline, RAMSETE, _log)
            self.timer.pause()

            counter += count
            logs.append(log)

            self.stopEventsHandler(stopEvents[len(logs)-1])

            self.timer.play()

        self.odometry.stop()

        print("Finished path")

        return logs, counter

    # @micropython.native
    def spline(self, path: list, RAMSETE: RAMSETEController, _log: bool = False) -> [list, int, dict]:
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

        if _log: log = []
        else: log = None

        index = 0

        while True:

            cTime = self.timer.get()

            waypoint, index = self.getTargetWaypoint(cTime, path, index)
            if waypoint is None: break

            currentX, currentY, currentTheata = self.odometry.getPos2d()
            Vx, Vy = waypoint[1] - currentX, waypoint[2] - currentY

            Vl, Vr = RAMSETE.correction(Vx, Vy, currentTheata,
                                        waypoint[4], waypoint[5], waypoint[3])

            self.drivebase.run_tankCM(Vl, Vr, waypoint[6], waypoint[7])

            if _log:
                log.append((cTime, currentX, currentY, currentTheata, self.drivebase.getSpeed(), Vl, Vr))

            count += 1

        self.drivebase.stop()
        return log, count

    # @micropython.native
    def stopEventsHandler(self, stopEvent: dict) -> None:
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
            st = time()
            self.commands(stopEvent["commands"], stopEvent["execBehavior"])
            sleep(stopEvent["waitTime"]-(time()-st))

    # @micropython.native
    def markersHandler(self, markers: tuple) -> None:
        """
        Handle markers.
        Parameters:
            markers: tuple - Markers
            cTime: float - Current time
        """
        sleep(markers[0][0]-self.timer.get())
        self.commands(markers[0][1], "Parallel")
        if len(markers > 1):
            self.markersHandler(markers[1:])

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
    def getTargetWaypoint(self, cTime: float, path: list, index: int) -> [list, int]:
        """
        Get target waypoint.
        Parameters:
            cTime: float - Current time
            path: list - Path
            index: int - Index
        Returns:
            waypoint: list - Waypoint
            index: int - Index
        """
        for waypoint in path[index:]:
            if waypoint[0] > cTime:
                return waypoint, index
            index += 1

        else:
            return None, index
