from abc import abstractmethod
from lib2to3.pgen2.token import OP
import logging
from typing import Callable, Optional, cast

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAVLink_heartbeat_message,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_STATE_STANDBY,
    EKF_ATTITUDE,
    EKF_VELOCITY_HORIZ,
    EKF_VELOCITY_VERT,
    EKF_POS_HORIZ_REL,
    EKF_POS_HORIZ_ABS,
    EKF_POS_VERT_ABS,
    EKF_POS_VERT_AGL,
    EKF_PRED_POS_HORIZ_REL,
    EKF_PRED_POS_HORIZ_ABS,
    MAVLink_statustext_message,
    MAVLink_ekf_status_report_message,
    MAVLink_local_position_ned_message,
)
from async_state_machine.client import Client, _ClientEventReaderFactory
from async_state_machine.transitions.combinators import all_of
from async_state_machine.transitions.types import (
    TransitionCheckerFactory,
    TransitionChecker,
)

from drones.commands import Arm, CommandSender, FlightMode, LocalPositionNED, SetFlightMode, Takeoff


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

def _is_flight_mode(message: MAVLink_message, mode: FlightMode) -> Optional[bool]:
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Hearbeat message: {message}")
    return FlightMode.GUIDED.value == cast(int, message.custom_mode)


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
    return cast(bool, message.flags == (
        EKF_ATTITUDE
        | EKF_VELOCITY_HORIZ
        | EKF_VELOCITY_VERT
        | EKF_POS_HORIZ_REL
        | EKF_POS_HORIZ_ABS
        | EKF_POS_VERT_ABS
        | EKF_PRED_POS_HORIZ_REL
        | EKF_PRED_POS_HORIZ_ABS
    ))


def _get_local_position(message: MAVLink_message) -> Optional[LocalPositionNED]:
    if message.get_msgId() != MAVLink_local_position_ned_message.id:
        return None
    assert isinstance(message, MAVLink_local_position_ned_message)
    logging.debug(f"Local position: {str(message)}")
    return LocalPositionNED(
        north=message.x,
        east=message.y,
        down=message.z,
    )


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
    
    def set_flight_mode(self, mode: FlightMode) -> TransitionCheckerFactory:
        return ActAndWaitForCheckerFactory(
            action_callback=lambda: self._commands_queue_tx.send(SetFlightMode(mode)),
            wait_for=self._event_client.when(lambda msg: _is_flight_mode(msg, mode)),
        )

    def when_local_position(
        self, condition: Callable[[LocalPositionNED], Optional[bool]]
    ) -> TransitionCheckerFactory:
        return self._event_client.when(
            lambda m: condition(cast(LocalPositionNED, _get_local_position(m)))
            if _get_local_position(m) is not None
            else None
        )

    def takeoff(
        self, height_m: float, allowed_error_m: float = 0.1
    ) -> TransitionCheckerFactory:
        return ActAndWaitForCheckerFactory(
            action_callback=lambda: self._commands_queue_tx.send(
                Takeoff(height_m=height_m)
            ),
            wait_for=self.when_local_position(
                lambda p: abs(height_m - (-p.down)) <= allowed_error_m
            ),
        )
