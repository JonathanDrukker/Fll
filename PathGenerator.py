import numpy as np
from json import load
from Tools.graph import log_graph
from math import atan2, sqrt
from time import time


def generatePathBezier(path: dict, STEP: float, TIMESTEP: float, DBM: float, scale):

    MAX_SPEED = path['maxVelocity']*scale
    MAX_ACCELARATION = path['maxAcceleration']*scale

    HALF_DBM = DBM/2

    spline = CubicBezierSpline2D(path['waypoints'], scale)

    reversed = -1 if path['isReversed'] else 1

    velocity_profile = VelocityProfile(MAX_SPEED, MAX_ACCELARATION, TIMESTEP, HALF_DBM)

    sVelocity = 0 if not path['waypoints'][0]['velOverride'] else path['waypoints'][0]['velOverride']

    x, y = spline.Coordinates(0, 0)
    spline_velocity = multiply_list(spline.Velocity(0, 0), reversed)

    theata = atan2(spline_velocity[1], spline_velocity[0])

    k = spline.Curvature(0, 0)

    accL, accR = velocity_profile.robot_velocity(
        MAX_ACCELARATION*reversed, velocity_profile.Omega(MAX_ACCELARATION*reversed, k))

    # [x, y, theata, V, omega, leftAccel, rightAccel]
    waypoints = [{'time': 0, 'waypoint': {'x': x, 'y': y, 'theata': theata,
                                          'V': sVelocity, 'omega': 0, 'leftAcc': accL, 'rightAcc': accR}}]

    velocity = velocity_profile.newVelocity(sVelocity, (accL+accR) / 2)
    omega = velocity_profile.newOmega(0, (accR-accL)*HALF_DBM)

    velMax, velOverload = None, None

    t = 1
    time = 0
    for u, waypoint in enumerate(path['waypoints'][:-1]):

        if waypoint['isReversal']:
            reversed = -reversed

        # (u, t, velocity)
        for _u, point in enumerate(path['waypoints'][u+1:]):
            _u += 1
            if point['velOverride']:
                velOverload = (u+_u, 0, point['velOverride'])
                velMax = None
                break
            elif point['isStopPoint'] or point['isReversal']:
                velOverload = (u+_u, 0, 0)
                velMax = None
                break
            elif (u+_u == len(path['waypoints'])-1):
                velOverload = (u+_u-1, 1, 0)
                velMax = None
                break
            else:
                deltaK = spline.Curvature(u+_u-1, 1) - spline.Curvature(u+_u, 0)
                if (deltaK < 0):
                    velOverload = None
                    velMax = (u+_u, 0, velocity_profile.curveVelocity(MAX_SPEED*reversed, k))
                    break

        print(u)

        t -= 1
        while t < 1:

            # TODO: uncomment
            t = spline.newT(u, t, STEP, abs(velocity*TIMESTEP))
            # t += 0.01

            time += TIMESTEP

            currentVelocity = velocity
            currentOmega = omega
            currentVl, currentVr = velocity_profile.robot_velocity(currentVelocity, currentOmega)

            x, y = spline.Coordinates(u, t)
            spline_velocity = multiply_list(spline.Velocity(u, t), reversed)

            theata = atan2(spline_velocity[1], spline_velocity[0])

            k = spline.Curvature(u, t, reversed)

            if velOverload:
                avgVel = (currentVelocity+velOverload[2])/2

                lengthLeft = spline.lengthBetween(u, t, velOverload[0], velOverload[1], STEP)
                lengthDecel = (currentVelocity - velOverload[2]) / MAX_ACCELARATION * avgVel
                if (lengthLeft > lengthDecel):
                    target = MAX_SPEED * reversed
                else:
                    target = velOverload[2]
            else:
                avgVel = (currentVelocity+velMax[2])/2

                lengthLeft = spline.lengthBetween(u, t, velMax[0], velMax[1], STEP)
                lengthDecel = abs((currentVelocity - velMax[2]) / MAX_ACCELARATION) * avgVel

                if (lengthLeft < lengthDecel):
                    target = MAX_SPEED * reversed
                else:
                    target = velMax[2]

            velocity = velocity_profile.curveVelocity(velocity_profile.Velocity(velocity, target), k)
            omega = velocity_profile.Omega(velocity, k)

            Vl, Vr = velocity_profile.robot_velocity(velocity, omega)
            accL, accR = velocity_profile.robot_acceleration(currentVl, currentVl, Vl, Vr)

            waypoints.append({'time': time, 'waypoint': {'x': x, 'y': y, 'theata': theata,
                                                         'V': velocity, 'omega': currentOmega,
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

    def Curvature(self, u: int, t: float, reverse: int = 1) -> float:
        return self.curves[u].Curvature(t, reverse)

    def lengthBetween(self, u1: int, t1: float, u2: int, t2: float, step: float) -> float:
        if (u1 - u2 == 0):
            return self.curves[u1].lengthBetween(t1, t2, step)
        length = 0
        for u in range(u1, u2+1):
            if (u == u1):
                length += self.curves[u].lengthBetween(t1, 1, step)
            elif (u == u2):
                length += self.curves[u].lengthBetween(0, t2, step)
            else:
                length += self.curves[u].lengthBetween(0, 1, step)
        return length

    def newT(self, u: float, t: float, step: float, dist: float) -> float:
        pastX, pastY = self.Coordinates(u, t)
        while dist > 0:
            t += step
            x, y = self.Coordinates(u, t)
            dist -= sqrt((x-pastX)**2 + (y-pastY)**2)
            pastX, pastY = x, y
        return t

    @staticmethod
    def _curvature(Velocity: list, Acceleration: list) -> float:
        return (Velocity[0]*Acceleration[1] - Velocity[1]*Acceleration[0]) / (Velocity[0]**2 + Velocity[1]**2)**(3/2)


class CubicBezierCurve2D:

    CHARACTERISTIC_MATRIX_CUBIC_BEZIER = np.array([[1, 0, 0, 0], [-3, 3, 0, 0], [3, -6, 3, 0], [-1, 3, -3, 1]])

    def __init__(self, points: np.ndarray):
        self.points = points

    def Coordinates(self, t: float) -> list:
        return (np.array([[1, t, t**2, t**3]]) @ self.CHARACTERISTIC_MATRIX_CUBIC_BEZIER @ self.points)[0, :]

    def Velocity(self, t: float, inverse: int = 1) -> list:
        return multiply_list((np.array([[0, 1, 2*t, 3*t**2]]) @
                              self.CHARACTERISTIC_MATRIX_CUBIC_BEZIER @ self.points)[0, :], inverse)

    def Acceleration(self, t: float, inverse: int = 1) -> list:
        return multiply_list((np.array([[0, 0, 2, 6*t]]) @
                             self.CHARACTERISTIC_MATRIX_CUBIC_BEZIER @ self.points)[0, :], inverse)

    def Curvature(self, t: float, inverse: int = 1) -> float:
        return self.__curvature(multiply_list(self.Acceleration(t), inverse), multiply_list(self.Velocity(t), inverse))

    def lengthBetween(self, t1: float, t2: float, step: float) -> float:
        length = 0
        lastX, lastY = self.Coordinates(t1)
        for t in np.arange(t1+step, t2, step):
            x, y = self.Coordinates(t)
            length += sqrt((x-lastX)**2 + (y-lastY)**2)
            lastX, lastY = x, y
        x, y = self.Coordinates(t2)
        length += sqrt((x-lastX)**2 + (y-lastY)**2)
        return length

    @staticmethod
    def __curvature(Velocity: list, Acceleration: list) -> float:
        return (Velocity[0]*Acceleration[1] - Velocity[1]*Acceleration[0]) / (Velocity[0]**2 + Velocity[1]**2)**(3/2)


class VelocityProfile:
    def __init__(self, maxVelocity: float, maxAcceleration: float, timestep: float, half_DBM):
        self.maxVelocity = maxVelocity
        self.maxAcceleration = maxAcceleration
        self.timestep = timestep
        self.half_DBM = half_DBM

    def Velocity(self, velocity, target):
        velocity += self.Acceleration(velocity, target) * self.timestep
        return min(abs(velocity), self.maxVelocity) * sign(velocity)

    def newVelocity(self, velocity, acceleration):
        return velocity + acceleration*self.timestep

    def Omega(self, velocity, k):
        return velocity * k

    def newOmega(self, omega, acceleration):
        return omega + acceleration*self.timestep

    def Acceleration(self, velocity, target):
        diff = target - velocity
        return min(self.maxAcceleration, abs(diff)) * sign(diff)

    def curveVelocity(self, velocity, k):
        return min(abs(velocity), sqrt(self.maxAcceleration*abs(1/k))) * sign(velocity)

    def robot_velocity(self, V, omega):
        return V - omega*self.half_DBM, V + omega*self.half_DBM

    def robot_acceleration(self, Vl0, Vr0, Vl1, Vr1):
        return self.__acceleration(Vl0, Vl1), self.__acceleration(Vr0, Vr1)

    def __acceleration(self, V0, V1):
        return (V1 - V0)/self.timestep

    def robot_velocity_limits(self, Vr, Vl):
        if (abs(Vl) > self.maxVelocity and abs(Vr) > self.maxVelocity):
            if (abs(Vl) == abs(Vr)):
                Vr = self.maxVelocity * sign(Vr)
                Vl = self.maxVelocity * sign(Vl)
            elif (abs(Vl) > abs(Vr)):
                ratio = Vr/Vl
                Vl = self.maxVelocity * sign(Vl)
                Vr = Vl * ratio
            else:
                ratio = Vl/Vr
                Vr = self.maxVelocity * sign(Vr)
                Vl = Vr * ratio
        elif (abs(Vl) > self.maxVelocity):
            ratio = Vr/Vl
            Vl = self.maxVelocity * sign(Vl)
            Vr = Vl * ratio
        elif (abs(Vr) > self.maxVelocity):
            ratio = Vl/Vr
            Vr = self.maxVelocity * sign(Vr)
            Vl = Vr * ratio

        if (abs(Vl) > self.maxVelocity):
            ratio = Vr/Vl
            Vl = self.maxVelocity * sign(Vl)
            Vr = Vl * ratio
        elif (abs(Vr) > self.maxVelocity):
            ratio = Vl/Vr
            Vr = self.maxVelocity * sign(Vr)
            Vl = Vr * ratio

        return Vl, Vr


def sign(num: int) -> int:
    if num == 0:
        return 0
    return 1 if num > 0 else -1


def multiply_list(list: list, num: int):
    return [i * num for i in list]


def main():

    DBM = 9.7
    WHEEL_RAD = 4.075

    STEP = 0.001
    TIMESTEP = 0.01

    SCALE = 100

    print('Starting...')

    with open('deploy/pathplanner/Test.path', 'r') as f:
        path = load(f)

    sTime = time()

    waypoints = generatePathBezier(path, STEP, TIMESTEP, DBM, SCALE)

    print(f'Generated path in {round(time()-sTime, 3)}s')

    log_graph(waypoints, WHEEL_RAD, DBM)

    print('Done!')


if __name__ == '__main__':
    main()
