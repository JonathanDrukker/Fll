import micropython
from _thread import start_new_thread
from time import time as _time


@micropython.native
def mean(*args: float) -> float:
    """
    This function returns the mean of the given numbers.

    Parameters: args: Numbers
    Returns: int: Mean
    """
    return sum(args)/len(args)


@micropython.native
def thread(func):
    """
    This decorator return the given function as a thread.

    Parameters: func: Function
    Returns: func: Thread
    """
    def wrapper(*args, **kwargs):
        start_new_thread(func, args, kwargs)
    return wrapper


@micropython.native
def sign(x: float) -> int:
    """
    This is a sign function.
    Returns -1 if x<0, 0 if x=0, 1 if x>0
    Parameters: x: float
    Returns: y
    """
    if x == 0: return 0
    return x/abs(x)


@micropython.native
def between(x: float, y: float, z: float):
    return abs(y-x) < z


class Timer:
    def __init__(self):
        self.st = _time()
        self.pt = _time()

    @micropython.native
    def get(self):
        """
        Return current time.
        """
        return _time() - self.st

    @micropython.native
    def reset(self):
        """
        Resets timer.
        """
        self.st = _time()

    @micropython.native
    def pause(self):
        """
        Pauses timer.
        """
        self.pt = _time()

    @micropython.native
    def play(self):
        """
        Plays/Unpauses timer.
        """
        self.st += _time() - self.pt
        self.pt = None
