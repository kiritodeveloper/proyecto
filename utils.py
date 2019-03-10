import time


def delay_until(time_to_delay):
    """
    Function used to get more accurate periods.
    It wait until some clock time
    :param time_to_delay: Clock time until we should wait
    """
    dt = time_to_delay - time.time()
    if dt > 0:
        time.sleep(dt)
