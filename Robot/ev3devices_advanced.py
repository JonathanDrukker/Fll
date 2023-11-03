from ev3devices_basic import Motor as BasicMotor


class Motor(BasicMotor):
    """
    Motor class
    Used to control the motor more easily, intuitionally and more functionally.
    Parameters:
        Port: str
        Direction: int
        bias: float
        acc: float
        speedKpid: [float, float, float]
        posKpid: [float, float, float]
    """
    def __init__(self, Port: str, Direction: int, bias: float = 0, acc: float = None,
                 speedKpid: [float, float, float] = [0, 0, 0],
                 posKpid: [float, float, float] = [0, 0, 0]):

        super().__init__(Port, Direction, bias)

        self.acc = acc

        self.speedKpid = speedKpid
        self.posKpid = posKpid

# TODO
