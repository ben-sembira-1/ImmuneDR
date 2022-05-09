import time
from abc import abstractmethod
from collections import deque
import logging
from math import nan
from typing import Deque, Optional, Protocol, Tuple

from pymavlink.mavextra import euler_to_quat
from pymavlink.mavutil import mavfile
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_GUIDED_CHANGE_ALTITUDE,
    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE,
)

from drones.mavlink_types import FlightMode


class Command(Protocol):
    @abstractmethod
    def __call__(self, mavlink_connection: mavfile) -> None:
        ...


class CommandReceiver:
    _inner_queue: Deque[Command]

    def __init__(self, inner_queue: Deque[Command]) -> None:
        self._inner_queue = inner_queue

    def receive(self) -> Optional[Command]:
        try:
            return self._inner_queue.popleft()
        except IndexError:
            return None


class CommandSender:
    _inner_queue: Deque[Command]

    def __init__(self, inner_queue: Deque[Command]) -> None:
        self._inner_queue = inner_queue

    def send(self, command: Command) -> None:
        self._inner_queue.append(command)


def create_command_queue_pair() -> Tuple[CommandSender, CommandReceiver]:
    inner_queue: Deque[Command] = deque()
    return (CommandSender(inner_queue), CommandReceiver(inner_queue))


class Arm(Command):
    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info("Executing Arm")
        mavlink_connection.arducopter_arm()


class Takeoff(Command):
    def __init__(
        self, height_m: float, yaw_angle: float = nan, ascend_rate_mps: float = 10.0
    ) -> None:
        super().__init__()
        self.height_m = height_m

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info("Executing takeoff")
        mav: MAVLink = mavlink_connection.mav
        mav.command_long_send(
            target_system=mavlink_connection.target_system,
            target_component=mavlink_connection.target_component,
            command=MAV_CMD_NAV_TAKEOFF,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=nan,
            param5=0.0,
            param6=0.0,
            param7=self.height_m,
        )


class ChangeAltitude(Command):
    def __init__(self, height_m: float, alt_rate_of_change: float = 10.0) -> None:
        """
        Change target altitude at a given rate. This slews the vehicle at a controllable rate between its previous
        altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands.)

        :param height_m: Target height in meters
        :param alt_rate_of_change: Rate of change, towards new altitude. 0 for maximum rate change.
        """
        super().__init__()
        self.height_m = height_m
        assert (
            alt_rate_of_change >= 0
        ), "Rate of change must be positive, negative numbers will not converge to target altitude!"
        self.alt_rate_of_change = alt_rate_of_change

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info(f"Ascending/descending to altitude {self.height_m}")
        mav: MAVLink = mavlink_connection.mav
        mav.command_long_send(
            target_system=mavlink_connection.target_system,
            target_component=mavlink_connection.target_component,
            command=MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=self.alt_rate_of_change,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=self.height_m,
        )


class SetAttitude(Command):
    HOLD_ALTITUDE_THRUST = (
        0.5  # Setting the thrust to this value tells holds a constant altitude
    )

    def __init__(self, heading: float, pitch: float, roll: float) -> None:
        """
        Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or
        other system).
        Primarily used in GUIDED_NOGPS mode.

        See https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-attitude-target
        and here https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET

        TODO should this hold altitude or not touch the thrust?
        """
        super().__init__()
        assert 0 <= heading < 360
        self.heading = heading
        self.pitch = pitch
        self.roll = roll
        self.attitude = [self.roll, self.pitch, self.heading]

    def __call__(self, mavlink_connection: mavfile) -> None:
        mav: MAVLink = mavlink_connection.mav
        mav.set_attitude_target_send(
            time_boot_ms=int(time.monotonic() * 1000),
            target_system=mavlink_connection.target_system,
            target_component=mavlink_connection.target_component,
            type_mask=ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            | ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE,
            q=euler_to_quat(self.attitude),
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=self.HOLD_ALTITUDE_THRUST,
        )


class SetFlightMode(Command):
    def __init__(self, mode: FlightMode) -> None:
        super().__init__()
        self.mode = mode

    def __call__(self, mavlink_connection: mavfile) -> None:
        logging.info(f"Setting flight mode to {self.mode.name}")
        mavlink_connection.set_mode(self.mode.value)
