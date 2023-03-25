from math import sin, cos, radians


class Position:
    def __init__(self, x, y, theata):
        self.x = x
        self.y = y
        self.theata = theata

    def __str__(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + \
             ", theata: " + str(self.theata)

    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        self.theata += other.theata

        self.theata %= 360
        if (self.theata < 0):
            self.theata += 360

        return self

    def __change__(self, v, omega):
        self.theata += omega
        self.theata %= 360

        if (self.theata < 0):
            self.theata += 360

        rad_theata = radians(self.theata)

        self.x += v*cos(rad_theata)
        self.y += v*sin(rad_theata)

        return self
