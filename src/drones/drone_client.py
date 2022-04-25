from threading import Event, Thread

import pymavlink.mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message
from async_state_machine.transitions.types import TransitionCheckerFactory, TransitionChecker
from async_state_machine.client import Client, _ClientEventReaderFactory


def _receive_messages_loop(
    mavlink_connection: pymavlink.mavutil.mavfile,
    sender_client: Client[MAVLink_message],
    stop_loop: Event,
):
    while not stop_loop.is_set():
        msg = mavlink_connection.recv_msg()
        if msg is not None:
            sender_client.to_state_machine(msg)

    mavlink_connection.close()

# class _DroneHeartbeatCheckerFactory(TransitionCheckerFactory):
#     def __init__(self) -> None:
#         super().__init__()
#     def create_checker(self) -> "TransitionChecker":
#         return 
class DroneClient:
    _event_client: _ClientEventReaderFactory[MAVLink_message]

    def __init__(
        self,
        event_client: _ClientEventReaderFactory[MAVLink_message]
    ) -> None:
        self._event_client = event_client

    def heartbeat(self):
        return self._event_client.when(lambda m: m.get_type() == 'HEARTBEAT')

    def arm(self):
        pass

class DroneDaemon:
    _mavlink_connection: pymavlink.mavutil.mavfile
    _sender_client: Client[MAVLink_message]
    _stop_loop_event: Event

    def __init__(self, mavlink_connection: pymavlink.mavutil.mavfile) -> None:
        self._mavlink_connection = mavlink_connection
        self._sender_client = Client()
        self._stop_loop_event = Event()
        self._thread = Thread(
            target=_receive_messages_loop,
            kwargs=dict(
                mavlink_connection=self._mavlink_connection,
                sender_client=self._sender_client,
                stop_loop=self._stop_loop_event,
            ),
        )
        self._thread.start()
    
    def create_client(self) -> DroneClient:
        return DroneClient(self._sender_client.events())
    
    def __del__(self):
        self._stop_loop_event.set()

