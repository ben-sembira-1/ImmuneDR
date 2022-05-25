import logging
import math
import time
from datetime import timedelta
from typing import Callable, Optional, TypeVar, Union, cast

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
    EKF_PRED_POS_HORIZ_REL,
    EKF_PRED_POS_HORIZ_ABS,
    MAVLink_statustext_message,
    MAVLink_ekf_status_report_message,
    MAVLink_local_position_ned_message,
    MAVLink_attitude_message,
    MAVLink_global_position_int_message,
)
from pymavlink.mavextra import angle_diff as untyped_angle_diff

angle_diff = cast(Callable[[float, float], float], untyped_angle_diff)

from async_state_machine.client import _ClientEventReaderFactory
from async_state_machine.transitions import timeout
from async_state_machine.transitions.combinators import all_of
from async_state_machine.transitions.types import (
    TransitionCheckerFactory,
    TransitionChecker,
)

from drones.commands import (
    Arm,
    CommandSender,
    SetFlightMode,
    Takeoff,
    SetAttitude,
    SetThrottle,
)
from drones.mavlink_types import (
    FlightMode,
    LocalPositionNED,
    AttitudeMessage,
    GlobalPositionInt,
)


def _is_heartbeat(message: MAVLink_message) -> bool:
    return cast(str, message.get_type()) != "HEARTBEAT"


def _is_armed(message: MAVLink_message) -> Optional[bool]:
    if isinstance(message, MAVLink_statustext_message):
        logging.debug(f"Message is: {message}")
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Heartbeat message: {message}")
    return cast(bool, (message.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0)


def _is_disarmed(message: MAVLink_message) -> Optional[bool]:
    is_armed = _is_armed(message)
    if is_armed is None:
        return None
    return not is_armed


def _is_flight_mode(message: MAVLink_message, mode: FlightMode) -> Optional[bool]:
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Heartbeat message: {message}")
    return mode.value == cast(int, message.custom_mode)


def _is_state_standby(message: MAVLink_message) -> Optional[bool]:
    if message.get_type() != "HEARTBEAT":
        return None
    assert isinstance(message, MAVLink_heartbeat_message)
    logging.debug(f"Hearbeat message: {message}")
    return cast(bool, message.system_status == MAV_STATE_STANDBY)


def _is_cancel_dr(message: MAVLink_message) -> Optional[bool]:
    # TODO
    return None


def _is_ekf_good(message: MAVLink_message) -> Optional[bool]:
    logging.debug(f"Message is: {message}")
    if message.get_type() != "EKF_STATUS_REPORT":
        return None
    assert isinstance(message, MAVLink_ekf_status_report_message)
    logging.debug(f"Ekf status report message message: {message}")
    return cast(
        bool,
        message.flags
        == (
            EKF_ATTITUDE
            | EKF_VELOCITY_HORIZ
            | EKF_VELOCITY_VERT
            | EKF_POS_HORIZ_REL
            | EKF_POS_HORIZ_ABS
            | EKF_POS_VERT_ABS
            | EKF_PRED_POS_HORIZ_REL
            | EKF_PRED_POS_HORIZ_ABS
        ),
    )


def _is_ekf_bad(message: MAVLink_message) -> Optional[bool]:
    ekf_good = _is_ekf_good(message)
    if ekf_good is None:
        return None
    return not ekf_good


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


def _get_global_position_int(message: MAVLink_message) -> Optional[GlobalPositionInt]:
    if message.get_msgId() != MAVLink_global_position_int_message.id:
        return None
    assert isinstance(message, MAVLink_global_position_int_message)
    logging.debug(f"Global position: {message}")
    return GlobalPositionInt.from_message(message)


def _get_attitude(message: MAVLink_message) -> Optional[AttitudeMessage]:
    if message.get_msgId() != MAVLink_attitude_message.id:
        return None
    assert isinstance(message, MAVLink_attitude_message)
    logging.debug(f"Attitude: {str(message)}")
    return AttitudeMessage(
        time_boot_ms=message.time_boot_ms,
        roll_rad=message.roll,
        pitch_rad=message.pitch,
        yaw_rad=message.yaw,
        roll_speed=message.rollspeed,
        pitch_speed=message.pitchspeed,
        yaw_speed=message.yawspeed,
    )


def _is_attitude_correct(
    current_attitude: AttitudeMessage,
    target_heading_deg: float,
    target_pitch_deg: float,
    target_roll_deg: float,
    max_error_deg: float,
) -> bool:
    return (
        abs(angle_diff(target_pitch_deg, current_attitude.pitch_deg)) <= max_error_deg
        and (
            abs(angle_diff(target_roll_deg, current_attitude.roll_deg)) <= max_error_deg
        )
        and (
            abs(angle_diff(target_heading_deg, current_attitude.yaw_deg))
            <= max_error_deg
        )
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


class ActUntilChecker(TransitionChecker):
    def __init__(
        self,
        action_callback: Callable[[], None],
        end_condition: TransitionCheckerFactory,
        action_interval: timedelta,
    ) -> None:
        super().__init__()
        self.action_callback = action_callback
        self.end_condition = end_condition.create_checker()
        self.action_interval = action_interval
        self.last_action_timestamp = -math.inf

    def poll_should_make_transition(self) -> Optional[bool]:
        should_end = self.end_condition.poll_should_make_transition()
        if should_end:
            return True
        current_time = time.monotonic()
        if (
            current_time - self.last_action_timestamp
            > self.action_interval.total_seconds()
        ):
            self.action_callback()
            self.last_action_timestamp = current_time
        return should_end


class ActUntilFactory(TransitionCheckerFactory):
    def __init__(
        self,
        action_callback: Callable[[], None],
        end_condition: TransitionCheckerFactory,
        action_interval: timedelta,
    ) -> None:
        super().__init__()
        self.action_callback = action_callback
        self.end_condition = end_condition
        self.action_interval = action_interval

    def create_checker(self) -> TransitionChecker:
        return ActUntilChecker(
            self.action_callback, self.end_condition, self.action_interval
        )


T = TypeVar("T")
R = TypeVar("R")


def apply_on_not_none(func: Callable[[T], R], m: Optional[T]) -> Optional[R]:
    if m is None:
        return None
    assert m is not None
    return func(m)


def maybe_lazy_get(
    maybe_lazy: Union[T, Callable[[], T]],
) -> T:
    return maybe_lazy() if callable(maybe_lazy) else maybe_lazy


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

    def ekf_good(self) -> TransitionCheckerFactory:
        return self._event_client.when(_is_ekf_good)

    def ekf_bad(self) -> TransitionCheckerFactory:
        return self._event_client.when(_is_ekf_bad)

    def preflight_finished(self) -> TransitionCheckerFactory:
        return all_of(
            [
                self._event_client.when(_is_state_standby),
                self.ekf_good(),
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
            lambda m: apply_on_not_none(condition, _get_local_position(m))
        )

    def when_global_position(
        self, condition: Callable[[GlobalPositionInt], Optional[bool]]
    ) -> TransitionCheckerFactory:
        return self._event_client.when(
            lambda m: apply_on_not_none(condition, _get_global_position_int(m))
        )

    def when_attitude(
        self, condition: Callable[[AttitudeMessage], Optional[bool]]
    ) -> TransitionCheckerFactory:
        return self._event_client.when(
            lambda m: apply_on_not_none(condition, _get_attitude(m))
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

    def turn(
        self,
        heading_deg: Union[float, Callable[[], float]],
        allowed_error_deg: float = 5.0,
    ) -> TransitionCheckerFactory:
        return self.set_attitude(
            heading_deg=heading_deg,
            max_error_deg=allowed_error_deg,
            pitch_deg=0,
            roll_deg=0,
        )

    def land(self) -> TransitionCheckerFactory:
        return ActAndWaitForCheckerFactory(
            action_callback=lambda: self._commands_queue_tx.send(
                SetFlightMode(FlightMode.LAND)
            ),
            wait_for=self._event_client.when(_is_disarmed),
        )

    def dr_cancelled(self) -> TransitionCheckerFactory:
        return self._event_client.when(_is_cancel_dr)

    def set_throttle(self, throttle: float) -> ActAndWaitForCheckerFactory:
        return ActAndWaitForCheckerFactory(
            action_callback=lambda: self._commands_queue_tx.send(SetThrottle(throttle)),
            wait_for=timeout(secs=0.1),
        )

    def set_attitude(
        self,
        pitch_deg: float,
        heading_deg: Union[float, Callable[[], float]],
        roll_deg: float = 0,
        end_condition: Optional[TransitionCheckerFactory] = None,
        max_error_deg: float = 3.0,
    ) -> ActUntilFactory:
        """
        Sends SetAttitudeTarget commands. These commands time out after a short interval, so to hold a steady attitude
        we need to keep resending the command.

        If no end_condition is passed, the checker will trigger when the target attitude is reached.
        If one is passed (e.g. a timeout), the command will be repeatedly sent until that condition is met.
        """
        end_condition = end_condition or self.when_attitude(
            lambda p: _is_attitude_correct(
                p,
                target_heading_deg=maybe_lazy_get(heading_deg),
                target_pitch_deg=pitch_deg,
                target_roll_deg=roll_deg,
                max_error_deg=max_error_deg,
            )
        )
        return ActUntilFactory(
            action_callback=lambda: self._commands_queue_tx.send(
                SetAttitude(
                    heading_deg=maybe_lazy_get(heading_deg),
                    pitch_deg=pitch_deg,
                    roll_deg=roll_deg,
                )
            ),
            end_condition=end_condition,
            action_interval=timedelta(seconds=0.1),
        )
