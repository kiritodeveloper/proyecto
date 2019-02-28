import time


def delay_until(time_to_delay):
    dt = time_to_delay - time.time()
    if dt > 0:
        time.sleep(dt)
