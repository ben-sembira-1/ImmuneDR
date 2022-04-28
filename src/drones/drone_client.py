from abc import abstractmethod
import logging
from typing import Callable, Optional, cast

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAVLink_heartbeat_message,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_STATE_STANDBY,
    EKF_ATTITUDE,
    MAVLink_statustext_message,
    MAVLink_ekf_status_report_message,
)
from async_state_machine.client import Client, _ClientEventReaderFactory
from async_state_machine.transitions.combinators import all_of
from async_state_machine.transitions.types import (
    TransitionCheckerFactory,
    TransitionChecker,
)

from drones.commands import Arm, CommandSender


def _is_heartbeat(message: MAVLink_message) -> bool:
    return cast(str, message.get_type()) != "HEARTBEAT"
    

def _is_armed(message: MAVLink_message) -> Optional[bool]:
    if isinstance(message, MAVLink_statustext_message):
        logging.debug(f"Message is: {message}")
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Hearbeat message: {message}")
    return cast(bool, (message.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0)


def _is_state_standby(message: MAVLink_message) -> Optional[bool]:
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Hearbeat message: {message}")
    return cast(bool, message.system_status == MAV_STATE_STANDBY)


def _is_ekf_good(message: MAVLink_message) -> Optional[bool]:
    logging.debug(f"Message is: {message}")
    if message.get_type() != "EKF_STATUS_REPORT":
        return None
    assert isinstance(message, MAVLink_ekf_status_report_message)
    logging.debug(f"Ekf status report message message: {message}")
    return cast(bool, (message.flags & EKF_ATTITUDE) != 0)


class ActAndWaitForCheckerFactory(TransitionCheckerFactory):
    def __init__(
        self, action_callback: Callable[[], None], wait_for: TransitionCheckerFactory
    ) -> None:
        super().__init__()
        self.action_callback = action_callback
        self.wait_for = wait_for

    def create_checker(self) -> TransitionChecker:
        logging.debug("Doing action before waiting for event")
        self.action_callback()
        return self.wait_for.create_checker()


class DroneClient:
    _event_client: _ClientEventReaderFactory[MAVLink_message]
    _commands_queue_tx: CommandSender

    def __init__(
        self,
        event_client: _ClientEventReaderFactory[MAVLink_message],
        commands_queue_tx: CommandSender,
    ) -> None:
        self._event_client = event_client
        self._commands_queue_tx = commands_queue_tx

    def heartbeat(self) -> TransitionCheckerFactory:
        return self._event_client.when(_is_heartbeat)

    def preflight_finished(self) -> TransitionCheckerFactory:
        return all_of(
            [
                self._event_client.when(_is_state_standby),
                self._event_client.when(_is_ekf_good),
            ]
        )

    def arm(self) -> TransitionCheckerFactory:
        return ActAndWaitForCheckerFactory(
            action_callback=lambda: self._commands_queue_tx.send(Arm()),
            wait_for=self._event_client.when(_is_armed),
        )
