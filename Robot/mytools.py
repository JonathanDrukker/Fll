import micropython
from _thread import start_new_thread
from time import time as _time
from time import sleep as _sleep


# @micropython.native
def mean(*args: float) -> float:
    """
    This function returns the mean of the given numbers.

    Parameters: args: Numbers
    Returns: int: Mean
    """
    return sum(args)/len(args)


# @micropython.native
def thread(func):
    """
    This decorator return the given function as a thread.

    Parameters: func: Function
    Returns: func: Thread
    """
    def wrapper(*args, **kwargs):
        start_new_thread(func, args, kwargs)
    return wrapper


# @micropython.native
async def sleep(time: float):
    """
    This function sleeps for the given time.

    Parameters: time: float - Time
    """
    _sleep(time)


class Timer:
    def __init__(self):
        self.st = _time()
        self.pt = None

    # @micropython.native
    def get(self):
        return _time() - self.st

    # @micropython.native
    def reset(self):
        self.st = _time()

    # @micropython.native
    def pause(self):
        self.pt = _time()

    # @micropython.native
    def play(self):
        self.st += _time() - self.pt
        self.pt = None
