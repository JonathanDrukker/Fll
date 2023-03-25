from _thread import start_new_thread


def mean(*args: float) -> float:
    """
    This function returns the mean of the given numbers.

    Parameters: Floats
    Returns: Mean-float
    """

    return sum(args)/len(args)


def sign(x: int):
    """
    This function returns the sign of the given number.

    Parameters: Number
    Returns: Sign
    """

    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


def thread(func):
    """
    This function is a decorator for threading a function.

    Parameters: Function
    Returns: Threaded function
    """

    def wrapper(*args, **kwargs):
        start_new_thread(func, args, kwargs)
    return wrapper
