from Tools.graph import log_graph
from pybricks.parameters import Port, Direction
from Robot.drivebase import DriveBase
from Robot.controllers import RAMSETEController
from math import pi, sin, cos, degrees
from random import choice
from Robot.robots import main_robot as robot_charecteristics


class Robot(DriveBase):
    def __init__(self, x, y, theata, wheelRad, DBM, lBias, rBias):

        super().__init__((Port.A, Direction.CLOCKWISE),
                         (Port.A, Direction.CLOCKWISE),
                         (Port.S1, Direction.COUNTERCLOCKWISE,
                         Port.S1, Direction.COUNTERCLOCKWISE),
                         wheelRad, DBM, x, y, theata)

        self.lBiasList = [i/1000 for i in range(1000)
                          for _ in range(int((-(i/1000-lBias)**2 + lBias**2) * 1000))]
        self.rBiasList = [i/1000 for i in range(1000)
                          for _ in range(int((-(i/1000-rBias)**2 + rBias**2) * 1000))]

    def trackPath(self, path, b, zeta, _print=False, _logVar=None):

        self.resetPos(path[0]['x'], path[0]['y'], path[0]['theata'])
        RAMSETE = RAMSETEController(b, zeta, self.halfDBM)

        for index, waypoint in enumerate(path[1:]):
            Dt = waypoint['time'] - path[index]['time']

            Vx, Vy = waypoint['x'] - self.x, waypoint['y'] - self.y

            Vl, Vr = RAMSETE.correction(Vx, Vy, self.theata,
                                        waypoint['V'], waypoint['omega'], waypoint['theata'])

            self.drive(self.motorSpeed(Vl), self.motorSpeed(Vr), Dt)

            data = {'time': waypoint['time'],
                    'robot': {
                    'Pose': (self.x, self.y, self.theata),
                    'gyroRate': degrees(self.omega),
                    'V': (self.Vl*360, self.Vr*360),
                    'Vtarget': (Vl/self.wheelCircumference*360, Vr/self.wheelCircumference*360)},
                    'waypoint': waypoint}

            if _print:
                print(data)
            if _logVar is not None:
                _logVar.append(data)

    def drive(self, Vl, Vr, t):

        self.Vl = Vl - 0.01*Vl*choice(self.lBiasList)
        self.Vr = Vr - 0.01*Vr*choice(self.rBiasList)

        Dl, Dr = Vl*t, Vr*t

        self.Velocity = self.wheelCircumference/2*(Dl+Dr)
        self.omega = self.wheelCircumference / self.DBM * (Dr-Dl)

        theata = self.theata + self.omega
        if theata > pi:
            theata -= 2*pi
        elif theata < -pi:
            theata += 2*pi

        self.x += self.Velocity*cos(theata)
        self.y += self.Velocity*sin(theata)
        self.theata = theata

    def motorSpeed(self, V):
        return V/self.wheelCircumference

    def resetPos(self, x, y, theata):
        self.x, self.y, self.theata = x, y, theata

    def odometry(self):
        pass


def main():
    print("Starting...")

    robot = Robot(0, 0, 0, robot_charecteristics.wheelRad, robot_charecteristics.DBM, 0.602, 0.512)

    with open('Robot/Test.waypoints', 'r') as file:
        path = eval(file.read())

    log = []
    robot.trackPath(path, 1, 1, _print=False, _logVar=log)
    log_graph(log, robot.wheelDiameter, robot.DBM)

    print("Done!")


if __name__ == "__main__":
    main()
