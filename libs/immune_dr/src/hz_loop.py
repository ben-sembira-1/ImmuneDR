import logging
import threading
import time
from typing import Generator, List


class HzLoop:
    def __init__(self, hz: int):
        self._routines: List[Generator] = []
        self._hz = hz
        self._routine_length = 1 / hz
        self._loop_thread = threading.Thread(target=self._loop)
        self._running = False
        self._logger = logging.getLogger(__name__)

    def add_routine(self, routine: Generator):
        self._routines.append(routine)

    def start(self):
        self._running = True
        self._loop_thread.start()

    def stop(self):
        self._running = False
        self._loop_thread.join(1)

    def _loop(self):
        next_wakeup_time = time.time()
        while self._running:
            for r in self._routines:
                try:
                    next(r)
                except StopIteration:
                    self._routines.remove(r)
            
            next_wakeup_time += self._routine_length
            if next_wakeup_time - time.time() < 0:
                self._logger.warning(
                    f"The routine takes to long to execute. Unable to run at {self._hz} HZ")
                # Make sure not to preserve an error in the timing loop.
                # If the loop did not manage to run at the requested speed do not try to overtake the delay.
                next_wakeup_time = time.time()
            time.sleep(max(next_wakeup_time - time.time(), 0))
            
