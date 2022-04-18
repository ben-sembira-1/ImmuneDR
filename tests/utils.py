import time


def wait(seconds: float, absolute: bool = False) -> None:
    if absolute:
        time.sleep(seconds)
    else:
        time.sleep(seconds / config.SPEED_UP)

def time_since(base_time: float, absolute: bool = False) -> float:
    elapsed = time.time() - base_time
    return elapsed if absolute else elapsed * config.SPEED_UP
