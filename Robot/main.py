#!/usr/bin/env pybricks-micropython

from ev3devices import Motor, Gyro
from pybricks.parameters import Port, Direction
from math import sin, cos, degrees, pi


lm = Motor(Port.D, Direction.CLOCKWISE)
rm = Motor(Port.C, Direction.CLOCKWISE)
gyro = Gyro(Port.S4, Direction.COUNTERCLOCKWISE)

r, d = 8.16/2, 9
c = 2*pi*r

x, y, theata = 0, 0, 0

past_lm_pos = lm.getRawAngle() / 360
past_rm_pos = rm.getRawAngle() / 360

while True:

    lm_pos = lm.getRawAngle() / 360
    rm_pos = rm.getRawAngle() / 360

    delta_lm_pos = lm_pos - past_lm_pos
    delta_rm_pos = rm_pos - past_rm_pos

    ds = c/2*(delta_rm_pos + delta_lm_pos)
    dTheata = c/d*(delta_rm_pos - delta_lm_pos)

    theata = theata + dTheata
    x = x + cos(theata)*ds
    y = y + sin(theata)*ds

    print("x: ", x, "y: ", y, "theta: ", degrees(theata))

    past_lm_pos = lm_pos
    past_rm_pos = rm_pos
