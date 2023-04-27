from Tools.graph import graph
from pybricks.parameters import Port, Direction
from Robot.drivebase import DriveBase
from math import pi, sin, cos
from random import choice


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

    def trackPath(self, path, b, zeta, _print=False, _graph=False):

        self.resetPos(path[0]['x'], path[0]['y'], path[0]['theata'])

        if _graph:
            Xr, Yr = [], []
            Xw, Yw = [], []

        for index, waypoint in enumerate(path[1:]):
            Dt = waypoint['time'] - path[index]['time']

            Vx, Vy = waypoint['x'] - self.x, waypoint['y'] - self.y

            Vl, Vr = self.calc_velocity(Vx, Vy, self.theata, self.halfDBM,
                                        waypoint['V'], waypoint['omega'], b, zeta)

            self.drive(self.motorSpeed(Vl), self.motorSpeed(Vr), Dt)

            if (_print):
                print("Pos:", self.x, self.y, self.theata,
                      "\nTarget:", waypoint['x'], waypoint['y'],
                      "\nVx:", Vx, "Vy:", Vy, "\nVl:", Vl, "Vr:", Vr)

            if _graph:
                Xr.append(self.x)
                Yr.append(self.y)
                Xw.append(waypoint['x'])
                Yw.append(waypoint['y'])

        if _graph:
            graph({'xr': Xr, 'yr': Yr, 'xw': Xw, 'yw': Yw, 'cr': 'hsv', 'cw': 'hsv',
                   'title': 'Simulation', 'label': ('X', 'Y')})

    def drive(self, Vl, Vr, t):

        Vl -= 0.01*Vl*choice(self.lBiasList)
        Vr -= 0.01*Vr*choice(self.rBiasList)

        Vl, Vr = Vl*t, Vr*t

        V = self.wheelCircumference/2*(Vr + Vl)
        omega = self.wheelCircumference / self.DBM * (Vr - Vl)

        theata = self.theata + omega
        if theata > pi:
            theata -= 2*pi
        elif theata < -pi:
            theata += 2*pi

        self.x += V*cos(theata)
        self.y += V*sin(theata)
        self.theata = theata

    def motorSpeed(self, V):
        return V/self.wheelCircumference

    def resetPos(self, x, y, theata):
        self.x, self.y, self.theata = x, y, theata

    def updatePosition(self):
        pass


def main():
    print("Starting...")

    robot = Robot(0, 0, 0, 8.16/2, 9.5, 0.602, 0.512)

    with open('Robot/Test.waypoints', 'r') as file:
        path = eval(file.read())

    robot.trackPath(path, 1, 1, _print=True, _graph=True)

    print("Done!")


if __name__ == "__main__":
    main()
