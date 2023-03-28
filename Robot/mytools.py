from _thread import start_new_thread


def mean(*args: float) -> float:
    """
    This function returns the mean of the given numbers.

    Parameters: Numbers
    Returns: Mean
    """
    return sum(args)/len(args)


def thread(func):
    """
    This decorator runs the given function in a new thread.

    Parameters: Function
    Returns: Thread
    """
    def wrapper(*args, **kwargs):
        start_new_thread(func, args, kwargs)
    return wrapper
