from dataclasses import dataclass


@dataclass
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
        theata = self.theata + other.theata

        theata %= 360

        if (theata < 0):
            theata += 360

        self.theata = theata

        return self
