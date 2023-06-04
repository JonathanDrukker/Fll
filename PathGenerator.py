import numpy as np
from json import load
from Tools.graph import log_graph
from math import atan2, sqrt


def generatePathBezier(path: dict, STEP: float, TIMESTEP: float, DBM: float):

    MAX_SPEED = path['maxVelocity']
    MAX_ACCELARATION = path['maxAcceleration']

    HALF_DBM = DBM/2

    spline = CubicBezierSpline2D(path['waypoints'], 100)

    reversed = -1 if path['isReversed'] else 1

    x, y = spline.Coordinates(0, 0)
    velocityX, velocityY = spline.Velocity(0, 0)
    accX, accY = spline.Acceleration(0, 0)
    velocityX, velocityY, accX, accY = velocityX*reversed, velocityY*reversed, accX*reversed, accY*reversed

    theata = atan2(velocityY, velocityX)
    velocity = 0 if not path['waypoints'][0]['velOverride'] else path['waypoints'][0]['velOverride']

    k = (velocityX*accY - velocityY*accX) / (velocityX**2 + velocityY**2)**(3/2)

    accL = MAX_ACCELARATION*reversed - k*MAX_ACCELARATION*HALF_DBM*reversed
    accR = MAX_ACCELARATION*reversed + k*MAX_ACCELARATION*HALF_DBM*reversed
    if abs(accL) > MAX_ACCELARATION and abs(accL) > abs(accR):
        ratio = accR/accL
        accL = MAX_ACCELARATION*reversed
        accR = accL*ratio
    elif abs(accR) > MAX_ACCELARATION and abs(accR) > abs(accL):
        ratio = accL/accR
        accR = MAX_ACCELARATION*reversed
        accL = accR*ratio

    # [x, y, theata, V, omega, leftAccel, rightAccel]
    waypoints = [{'time': 0, 'waypoint': {'x': x, 'y': y, 'theata': theata,
                                          'V': velocity, 'omega': 0, 'leftAcc': accL, 'rightAcc': accR}}]

    u = 0
    t = 1
    time = 0
    for index, waypoint in enumerate(path['waypoints'][:-1]):

        reversed = -reversed if waypoint['isReversal'] else reversed

        # (u, velocity)
        for _index, _waypoint in enumerate(path['waypoints'][index:]):
            if _waypoint['isStopPoint'] or _waypoint['isReversal']:
                velOverload = (index+_index+1, 0)
                velMaxEnd = None
                break
            elif _waypoint['velOverride']:
                velOverload = (index+_index+1, _waypoint['velOverride'])
                velMaxEnd = None
                break
            elif (_index+index+1) == len(path['waypoints']):
                velOverload = (index+_index+1, 0)
                velMaxEnd = None
                break
            else:
                _past_velocity = spline.Velocity(index+_index, 1)
                _past_velocity = [i*reversed for i in _past_velocity]
                _past_acc = spline.Acceleration(index+_index, 1)
                _past_acc = [i*reversed for i in _past_acc]
                _pastK = spline.Curvature(_past_velocity, _past_acc)

                _velocity = spline.Velocity(index+_index+1, 0)
                _velocity = [i*reversed for i in _velocity]
                _acc = spline.Acceleration(index+_index+1, 0)
                _acc = [i*reversed for i in _acc]
                _k = spline.Curvature(_velocity, _acc)

                deltaK = _k - _pastK

                if (deltaK > 0):
                    vel = sqrt(MAX_ACCELARATION*abs(1/k))
                    velOverload = None
                    velMaxEnd = (index+_index+1, vel)
                    break

        t -= 1
        while t < 1:

            # deltaDist = velocity*TIMESTEP
            # u, t = spline.findT(u, t, STEP, deltaDist)
            t += 0.01

            time += TIMESTEP

            lastWaypoint = waypoints[-1]['waypoint']

            currentVelocity = lastWaypoint['V'] + (lastWaypoint['leftAcc'] + lastWaypoint['rightAcc'])/2*TIMESTEP
            currentOmega = lastWaypoint['omega'] + (lastWaypoint['rightAcc'] - lastWaypoint['leftAcc']) / HALF_DBM * TIMESTEP
            currentVl, currentVr = currentVelocity - currentOmega*HALF_DBM, currentVelocity + currentOmega*HALF_DBM

            x, y = spline.Coordinates(index, t)
            velocityX, velocityY = spline.Velocity(index, t)
            accX, accY = spline.Acceleration(index, t)

            velocityX, velocityY, accX, accY = velocityX*reversed, velocityY*reversed, accX*reversed, accY*reversed

            theata = atan2(velocityY, velocityX)
            k = spline.Curvature((velocityX, velocityY), (accX, accY))

            acceleration = MAX_ACCELARATION * reversed if abs(velocity) + MAX_ACCELARATION * \
                TIMESTEP < MAX_SPEED else (MAX_SPEED - abs(velocity))/TIMESTEP * reversed

            if velOverload:
                avgVel = ((velOverload[1] + velocity) / 2)
                if (avgVel == 0):
                    velocity += acceleration*TIMESTEP
                else:

                    length = spline.lengthBetween(int(u), u-int(u), int(velOverload[0]),
                                                  velOverload[0]-int(velOverload[0]), 0.01)

                    time_deccel = (velocity - velOverload[1]) / MAX_ACCELARATION
                    time_left = length / avgVel

                    if (time_deccel < time_left):
                        acceleration = MAX_ACCELARATION * reversed if abs(velocity) + MAX_ACCELARATION * \
                            TIMESTEP < MAX_SPEED else (MAX_SPEED - abs(velocity))/TIMESTEP * reversed
                        velocity += acceleration*TIMESTEP
                    else:
                        acceleration = MAX_ACCELARATION * reversed if abs(velocity) + MAX_ACCELARATION * \
                            TIMESTEP < velOverload[1] else (velOverload[1] - abs(velocity))/TIMESTEP * reversed
                        velocity -= acceleration*TIMESTEP
            else:
                if (velocity < velMaxEnd[1]):
                    acceleration = MAX_ACCELARATION * reversed if abs(velocity) + MAX_ACCELARATION * \
                        TIMESTEP < MAX_SPEED else (MAX_SPEED - abs(velocity))/TIMESTEP * reversed
                    velocity += acceleration*TIMESTEP
                else:

                    avgVel = ((velMaxEnd[1] + velocity) / 2)
                    if (avgVel == 0):
                        velocity += acceleration*TIMESTEP
                    else:

                        length = spline.lengthBetween(int(u), u-int(u), int(velMaxEnd[0]),
                                                      velMaxEnd[0]-int(velMaxEnd[0]), 0.01)

                        time_deccel = (velocity - velMaxEnd[1]) / MAX_ACCELARATION
                        time_left = length / avgVel

                        if (time_deccel < time_left):
                            acceleration = MAX_ACCELARATION * reversed if abs(velocity) + MAX_ACCELARATION * \
                                TIMESTEP < MAX_SPEED else (MAX_SPEED - abs(velocity))/TIMESTEP * reversed
                            velocity += acceleration*TIMESTEP
                        else:
                            acceleration = MAX_ACCELARATION * reversed if abs(velocity) + MAX_ACCELARATION * \
                                TIMESTEP < velMaxEnd[1] else (velMaxEnd[1] - abs(velocity))/TIMESTEP * reversed
                            velocity -= acceleration*TIMESTEP

            velocity = min(abs(velocity), sqrt(MAX_ACCELARATION*abs(1/k))) * reversed

            omega = k * velocity
            Vl, Vr = velocity - omega*HALF_DBM, velocity + omega*HALF_DBM

            accL, accR = (Vl - currentVl)/TIMESTEP, (Vr - currentVr)/TIMESTEP

            waypoints.append({'time': time, 'waypoint': {'x': x, 'y': y, 'theata': theata,
                                                         'V': currentVelocity, 'omega': currentOmega,
                                                         'leftAcc': accL, 'rightAcc': accR}})

    return waypoints


class CubicBezierSpline2D:
    def __init__(self, points: list, scale: float = 1):
        self.points = points
        self.unitChangeRatio = scale

        self.curves = []
        for index, point in enumerate(points[:-1]):

            anchor = list(point['anchorPoint'].values())
            control = list(point['nextControl'].values())
            nextControl = list(points[index+1]['prevControl'].values())
            nextAnchor = list(points[index+1]['anchorPoint'].values())

            self.curves.append(CubicBezierCurve2D(scale*np.array([anchor, control,
                                                                  nextControl, nextAnchor])))

    def Coordinates(self, u: int, t: float) -> list:
        return self.curves[u].Coordinates(t)

    def Velocity(self, u: int, t: float) -> list:
        return self.curves[u].Velocity(t)

    def Acceleration(self, u: int, t: float) -> list:
        return self.curves[u].Acceleration(t)

    def lengthBetween(self, u1: int, t1: float, u2: int, t2: float, step: float) -> float:
        if (u1 - u2 == 0):
            return self.curves[u1].lengthBetween(t1, t2, step)
        length = 0
        for u in range(u1, u2):
            if u == int(u1):
                length += self.curves[u].lengthBetween(t1, 1, step)
            elif u == int(u2):
                length += self.curves[u].lengthBetween(0, t2, step)
            else:
                length += self.curves[u].lengthBetween(0, 1, step)
        return length

    def findT(self, u: float, t: float, step: float, dist: float) -> float:
        pastX, pastY = self.Coordinates(u, t)
        while dist > 0:
            t += step
            x, y = self.Coordinates(u, t+step)
            dist -= sqrt((x-pastX)**2 + (y-pastY)**2)
            pastX, pastY = x, y
        u += int(t)
        t -= int(t)
        return u, t

    @staticmethod
    def Curvature(Velocity: list, Acceleration: list) -> float:
        return (Velocity[0]*Acceleration[1] - Velocity[1]*Acceleration[0]) / (Velocity[0]**2 + Velocity[1]**2)**(3/2)


class CubicBezierCurve2D:

    CHARACTERISTIC_MATRIX_CUBIC_BEZIER = np.array([[1, 0, 0, 0], [-3, 3, 0, 0], [3, -6, 3, 0], [-1, 3, -3, 1]])

    def __init__(self, points: np.ndarray):
        self.points = points

    def Coordinates(self, t: float) -> list:
        return (np.array([[1, t, t**2, t**3]]) @ self.CHARACTERISTIC_MATRIX_CUBIC_BEZIER @ self.points)[0, :]

    def Velocity(self, t: float) -> list:
        return (np.array([[0, 1, 2*t, 3*t**2]]) @ self.CHARACTERISTIC_MATRIX_CUBIC_BEZIER @ self.points)[0, :]

    def Acceleration(self, t: float) -> list:
        return (np.array([[0, 0, 2, 6*t]]) @ self.CHARACTERISTIC_MATRIX_CUBIC_BEZIER @ self.points)[0, :]

    def lengthBetween(self, t1: float, t2: float, step: float) -> float:
        length = 0.0
        lastX, lastY = self.Coordinates(t1)
        for t in np.arange(t1+step, t2, step):
            x, y = self.Coordinates(t)
            length += sqrt((x-lastX)**2 + (y-lastY)**2)
            lastX, lastY = x, y
        x, y = self.Coordinates(t2)
        length += sqrt((x-lastX)**2 + (y-lastY)**2)
        return length

    @staticmethod
    def Curvature(Velocity: list, Acceleration: list) -> float:
        return (Velocity[0]*Acceleration[1] - Velocity[1]*Acceleration[0]) / sqrt(Velocity[0]**2 + Velocity[1]**2)**3


def sign(num: int) -> int:
    if num == 0:
        return 0
    return 1 if num > 0 else -1


def main():

    DBM = 9.7
    WHEEL_RAD = 4.075

    STEP = 0.001
    TIMESTEP = 0.01

    print('Starting...')

    with open('deploy/pathplanner/Test.path', 'r') as f:
        path = load(f)

    waypoints = generatePathBezier(path, STEP, TIMESTEP, DBM)

    log_graph(waypoints, WHEEL_RAD, DBM)

    print('Done!')


if __name__ == '__main__':
    main()
