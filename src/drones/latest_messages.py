import copy
import logging
from threading import Event, Thread, Lock
import time
from typing import Dict, Optional, Type

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message
from async_state_machine.client import _ClientEventReader

from drones.drone_client import DroneClient


class LatestMessagesCache:
    _latest_events: Dict[Type[MAVLink_message], MAVLink_message]
    _latest_events_lock: Lock
    _stop_update_loop: Event
    _updater_thread: Thread

    def __init__(self, drone_client: DroneClient) -> None:
        self._latest_events = {}
        self._latest_events_lock = Lock()
        self._stop_update_loop = Event()
        self._updater_thread = Thread(
            target=self._update_loop,
            kwargs=dict(event_reader=drone_client._event_client.create_event_reader()),
            daemon=True,  # Let the __del__ close the thread, and not threading._shutdown - avoids deadlock
        )
        self._updater_thread.start()

    def set_latest_message(self, message: MAVLink_message) -> None:
        with self._latest_events_lock:
            self._latest_events[type(message)] = message

    def get_all_latest(self) -> Dict[Type[MAVLink_message], MAVLink_message]:
        with self._latest_events_lock:
            return copy.copy(self._latest_events)

    def get_latest_message(
        self, message_type: Type[MAVLink_message]
    ) -> Optional[MAVLink_message]:
        with self._latest_events_lock:
            return self._latest_events.get(message_type)

    def _update_loop(self, event_reader: _ClientEventReader[MAVLink_message]) -> None:
        while not self._stop_update_loop.is_set():
            message = event_reader.read_latest_event()
            if message is not None:
                self.set_latest_message(message)
            time.sleep(1e-10 if message is None else 0)

    def __del__(self) -> None:
        logging.debug("LatestMessageCache stopping thread...")
        self._stop_update_loop.set()
        self._updater_thread.join(10)
        if self._updater_thread.is_alive():
            logging.error("LatestMessageCache update looped thread timed out")
