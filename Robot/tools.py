from _thread import start_new_thread


def mean(*args: float) -> float:
    """
    This function returns the mean of the given numbers.

    Parameters: Floats
    Returns: Mean-float
    """

    return sum(args)/len(args)


def thread(func):
    """
    This function is a decorator for threading a function.

    Parameters: Function
    Returns: Threaded function
    """

    def wrapper(*args, **kwargs):
        start_new_thread(func, args, kwargs)
    return wrapper
