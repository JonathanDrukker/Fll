from math import pi


class RobotCharacteristics:
    def __init__(self, DBM, wheelRad, width, length, wheelAxis):
        self.DBM = DBM
        self.halfDBM = DBM / 2
        self.wheelRad = wheelRad
        self.wheelDiameter = wheelRad * 2
        self.wheelCircumference = wheelRad * 2 * pi
        self.width = width
        self.length = length
        self.wheelAxis = wheelAxis
        self.WA_to_middle = self.length/2 - self.wheelAxis


class Bruria(RobotCharacteristics):
    def __init__(self):
        super().__init__(9.7, 4.075, 13.5, 16.5, 5.8)


robot = Bruria()
