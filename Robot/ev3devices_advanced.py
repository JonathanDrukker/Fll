from ev3devices_basic import Motor as BasicMotor


class Motor(BasicMotor):
    def __init__(self, Port: str, Direction: int, bias: float = 0, acc: float = None,
                 speedKpid: [float, float, float] = [0, 0, 0],
                 posKpid: [float, float, float] = [0, 0, 0]):

        super().__init__(Port, Direction, bias)

        self.acc = acc

        self.speedKpid = speedKpid
        self.posKpid = posKpid

# TODO
