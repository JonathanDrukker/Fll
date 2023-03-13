#!/usr/bin/env pybricks-micropython

from pybricks.parameters import Port, Direction
from drivebase import DriveBase
from parameters import Position


print("Starting...")

drivebase = DriveBase((Port.D, Direction.CLOCKWISE),
                      (Port.B, Direction.CLOCKWISE),
                      (Port.S3, Direction.COUNTERCLOCKWISE,
                      Port.S4, Direction.COUNTERCLOCKWISE),
                      8.16/2, 9.5, Position(0, 0, 0))

drivebase.gyro.resetAngle(0)
drivebase.lm.resetAngle(0)
drivebase.rm.resetAngle(0)

print("Done!")
