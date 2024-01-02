from math import cos, sin, atan2, degrees
from json import loads


def calcCoords(x, y, Dx, Dy, length):

    length += 0.738 / 2

    theata = atan2(Dy, Dx)

    x += length * sin(theata)
    y += length * cos(theata)

    return x, y, degrees(theata)


def main():

    x, y = 16, 4.5
    Dx, Dy = 14.25, 10.25

    with open("Robot/config.json", "r") as f:
        config = loads(f.read())
        length = config["robot"]["length"]/2

        x, y, theata = calcCoords(x, y, Dx, Dy, length)

        print("Coords:", x, y, theata)


if __name__ == "__main__":
    main()
