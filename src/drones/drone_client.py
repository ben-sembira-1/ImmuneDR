import enum
import logging
from optparse import Option
from threading import Event, Thread
from collections import deque
from typing import Callable, Deque, Optional

import pymavlink.mavutil
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAVLink_heartbeat_message,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_STATE_STANDBY,
    EKF_ATTITUDE,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAVLink_statustext_message,
    MAVLink_ekf_status_report_message,
)
from async_state_machine.client import Client, _ClientEventReaderFactory
from async_state_machine.transitions.combinators import all_of
from async_state_machine.transitions.types import (
    TransitionCheckerFactory,
    TransitionChecker,
)


@enum.unique
class Commands(enum.Enum):
    ARM = "ARM"


MESSAGES_INTERVAL_US = {MAVLink_ekf_status_report_message: 100}


def _mavlink_control_loop(
    mavlink_connection: pymavlink.mavutil.mavfile,
    sender_client: Client[MAVLink_message],
    command_rx_queue: Deque[Commands],
    stop_loop: Event,
    target_system_id: int,
    target_component_id: int,
):

    try:
        from pymavlink.dialects.v20.ardupilotmega import MAVLink

        mav: MAVLink = mavlink_connection.mav
        for msg_type, interval in MESSAGES_INTERVAL_US.items():
            logging.info(f"Set message {msg_type.name} ({msg_type.id}) to {interval}[us]")
            mav.command_long_send(
                target_system=target_system_id,
                target_component=target_component_id,
                command=MAV_CMD_SET_MESSAGE_INTERVAL,
                confirmation=0,
                param1=msg_type.id,
                param2=float(interval),
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )

        logging.info(f"Starting mavlink message loop")
        while not stop_loop.is_set():
            msg = mavlink_connection.recv_msg()
            if msg is not None:
                sender_client.to_state_machine(msg)
            try:
                # TODO: Improve polymorphism of commands
                # and the queue interface and mechanism (not sending raw commands)
                cmd = command_rx_queue.popleft()
                if cmd is Commands.ARM:
                    mavlink_connection.arducopter_arm()
            except IndexError:
                pass
    except Exception as e:
        logging.error(f"Drone daemon loop failed: {e}")
        raise
    finally:
        mavlink_connection.close()


def _is_armed(message: MAVLink_message) -> Optional[bool]:
    if isinstance(message, MAVLink_statustext_message):
        logging.debug(f"Message is: {message}")
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Hearbeat message: {message}")
    return (message.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0


def _is_state_standby(message: MAVLink_message) -> Optional[bool]:
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Hearbeat message: {message}")
    return message.system_status == MAV_STATE_STANDBY


def _is_ekf_good(message: MAVLink_message) -> Optional[bool]:
    logging.debug(f"Message is: {message}")
    if message.get_type() != "EKF_STATUS_REPORT":
        return None
    assert isinstance(message, MAVLink_ekf_status_report_message)
    logging.debug(f"Ekf status report message message: {message}")
    return (message.flags & EKF_ATTITUDE) != 0


class ActAndWaitForCheckerFactory(TransitionCheckerFactory):
    def __init__(
        self, action_callback: Callable[[], None], wait_for: TransitionCheckerFactory
    ) -> None:
        super().__init__()
        self.action_callback = action_callback
        self.wait_for = wait_for

    def create_checker(self) -> TransitionChecker:
        logging.debug("Sending command...")
        self.action_callback()
        return self.wait_for.create_checker()


class DroneClient:
    _event_client: _ClientEventReaderFactory[MAVLink_message]
    _commands_queue_tx: Deque[Commands]

    def __init__(
        self,
        event_client: _ClientEventReaderFactory[MAVLink_message],
        commands_queue_tx: Deque[Commands],
    ) -> None:
        self._event_client = event_client
        self._commands_queue_tx = commands_queue_tx

    # TODO: The whole command communication should be refactored out
    def _send_command(self, command: Commands) -> None:
        self._commands_queue_tx.append(command)

    def heartbeat(self) -> TransitionCheckerFactory:
        return self._event_client.when(lambda m: m.get_type() == "HEARTBEAT")

    def preflight_finished(self) -> TransitionCheckerFactory:
        return all_of(
            [
                self._event_client.when(_is_state_standby),
                self._event_client.when(_is_ekf_good),
            ]
        )

    def arm(self) -> TransitionCheckerFactory:
        return ActAndWaitForCheckerFactory(
            action_callback=lambda: self._send_command(Commands.ARM),
            wait_for=self._event_client.when(_is_armed),
        )


class DroneDaemon:
    drone_system_id: int
    drone_component_id: int
    _mavlink_connection: pymavlink.mavutil.mavfile
    _sender_client: Client[MAVLink_message]
    _stop_loop_event: Event
    _command_queue: Deque[Commands]

    def __init__(
        self,
        mavlink_connection: pymavlink.mavutil.mavfile,
        drone_system_id: int = 1,
        drone_component_id: int = 1,
    ) -> None:
        self.drone_system_id = drone_system_id
        self.drone_component_id = drone_component_id
        self._mavlink_connection = mavlink_connection
        self._sender_client = Client()
        self._stop_loop_event = Event()
        self._command_queue = deque()
        self._thread = Thread(
            target=_mavlink_control_loop,
            kwargs=dict(
                mavlink_connection=self._mavlink_connection,
                sender_client=self._sender_client,
                stop_loop=self._stop_loop_event,
                command_rx_queue=self._command_queue,
                target_system_id=self.drone_system_id,
                target_component_id=self.drone_component_id,
            ),
        )
        self._thread.start()

    def create_client(self) -> DroneClient:
        return DroneClient(self._sender_client.events(), self._command_queue)

    def __del__(self):
        self._stop_loop_event.set()
        self._thread.join(10)
