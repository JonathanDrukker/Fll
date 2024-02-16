import micropython
from gc import collect
from sys import exit
from _thread import allocate_lock, start_new_thread
from mytools import thread, Timer
from time import time, sleep
from controllers import RAMSETEController
from odometry import DiffrentialDriveOdometry
from drivebase import DriveBase
from ev3devices_advanced import Motor
from sensorbase import Sensorbase


class Runner:
    """
    Runner class - Used to run paths.
    Controls everything in the robot.
    Parameters:
        config: dict
    """

    lock = allocate_lock()
    run = True

    def __init__(self, config: dict):

        self.drivebase = DriveBase(config)
        self.odometry = DiffrentialDriveOdometry(self.drivebase, self.lock)

        self.sensorbase = Sensorbase(config.sensorbase, config.light, self.drivebase)

        self.lm = Motor(config.motors.left)
        self.rm = Motor(config.motors.right)

    @micropython.native
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
        collect()

        print("Loading Path...")
        st = time()
        with open("/home/robot/Robot/Paths/"+filename+".path", "r") as f:
            path = eval(f.read())
        print("Loaded path in", time()-st)

        with open("/home/robot/Robot/Paths/"+filename+".events", "r") as f:
            stopEvents, markers = eval(f.readline())

        logs = []
        counter = 0

        self.RAMSETE = RAMSETEController(b, zeta, self.drivebase._halfDBM)

        self.odometry.resetPos(path[0][0][1], path[0][0][2], path[0][0][3])
        self.odometry.start()

        self.stopEventsHandler(stopEvents[0])

        self.timer = Timer()

        for index, spline in enumerate(path):

            self.markersHandler(markers[index])

            log, count = self.spline(spline, _log)
            self.timer.pause()

            counter += count
            logs.append(log)

            if not self.run:
                break

            self.stopEventsHandler(stopEvents[index+1])

            self.timer.play()

        self.odometry.stop()

        print("Finished path")

        return logs, counter

    @micropython.native
    def spline(self, path: list, _log: bool = False) -> [list, int]:
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

        if _log:
            log = []
        else:
            log = None

        index = 0

        while self.run:

            cTime = self.timer.get()

            waypoint, index = self.getTargetWaypoint(cTime, path, index)
            if waypoint is None:
                break

            currentX, currentY, currentTheata = self.odometry.getPos2d()
            Vx, Vy = waypoint[1] - currentX, waypoint[2] - currentY

            Vl, Vr = self.RAMSETE.correction(Vx, Vy, currentTheata,
                                             waypoint[4], waypoint[5], waypoint[3])

            self.drivebase.run_tankCM(Vl, Vr, waypoint[6], waypoint[7])

            if _log:
                log.append((cTime, currentX, currentY, currentTheata, self.drivebase.getSpeed(), Vl, Vr))

            count += 1

        self.drivebase.stop()
        return log, count

    @micropython.native
    def stopEventsHandler(self, stopEvent: dict) -> None:
        """
        Handle stop events.
        Parameters:
            stopEvent: dict - Stop event
        """
        if stopEvent["waitBehavior"] == "none":
            self.commands(stopEvent["commands"], stopEvent["executionBehavior"])
        elif stopEvent["waitBehavior"] == "before":
            sleep(stopEvent["waitTime"])
            self.commands(stopEvent["commands"], stopEvent["executionBehavior"])
        elif stopEvent["waitBehavior"] == "after":
            self.commands(stopEvent["commands"], stopEvent["executionBehavior"])
            sleep(stopEvent["waitTime"])
        elif stopEvent["waitBehavior"] == "minimum":
            st = time()
            self.commands(stopEvent["commands"], stopEvent["executionBehavior"])
            dt = stopEvent["waitTime"]-(time()-st)
            if dt > 0:
                sleep(dt)

    @micropython.native
    @thread
    def markersHandler(self, markers: list) -> None:
        """
        Handle markers.
        Parameters:
            markers: tuple - Markers
            cTime: float - Current time
        """
        if len(markers) > 0:
            dt = markers[0][0]-self.timer.get()
            if dt > 0:
                sleep(dt)
            self.commands(markers[0][1], "parallel")
            self.markersHandler(markers[1:])

    @micropython.native
    def commands(self, commands: list, execBehavior: str) -> None:
        """
        Execute commands.
        Parameters:
            commands: list - Commands
            execBehavior: str - Execution behavior
        """
        if execBehavior == "parallel":
            for command in commands:
                start_new_thread(eval, (command,))
        elif execBehavior == "sequential":
            for command in commands:
                eval(command)

    @micropython.native
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

    @micropython.native
    def exit(self):
        self.run = False
        self.drivebase.stopUpdate()
        self.lm.stopUpdate()
        self.rm.stopUpdate()
        self.odometry.stop()
        exit()

    @micropython.native
    def run2_M3(self):
        self.odometry.stop()
        self.drivebase.run_tank(-400, -400)
        sleep(0.75)
        self.odometry.start()
        self.rm.RunTime(1000, 2.5)
        self.rm.RunTime(-1000, 1, wait=False)

    @micropython.native
    def run4_M7(self):
        self.odometry.stop()
        self.drivebase.run_tank(1000, 1000)
        sleep(1)
        self.odometry.resetPos(174, 86, 45.00)
        self.odometry.start()
        self.rm.RunTime(-200, 2)
        self.lm.RunTime(-1000, 1.5)
